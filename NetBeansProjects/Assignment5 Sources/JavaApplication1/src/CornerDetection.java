
import java.awt.*;
import java.awt.event.*;
import java.awt.image.*;
import java.io.*;
import javax.imageio.*;
import javax.swing.*;

// Main class
public class CornerDetection extends Frame implements ActionListener {

    BufferedImage input;
    int width, height;
    double sensitivity = .1;
    int threshold = 20;
    ImageCanvas source, target;
    CheckboxGroup metrics = new CheckboxGroup();

    private float[][] Ix2;
    private float[][] Ixy;
    private float[][] Iy2;
    // Constructor

    public CornerDetection(String name) {
        super("Corner Detection");
        // load image
        try {
            input = ImageIO.read(new File(name));
        } catch (Exception ex) {
            ex.printStackTrace();
        }
        width = input.getWidth();
        height = input.getHeight();
        // prepare the panel for image canvas.
        Panel main = new Panel();
        source = new ImageCanvas(input);
        target = new ImageCanvas(width, height);
        main.setLayout(new GridLayout(1, 2, 10, 10));
        main.add(source);
        main.add(target);
        // prepare the panel for buttons.
        Panel controls = new Panel();
        Button button = new Button("Derivatives");
        button.addActionListener(this);
        controls.add(button);
        // Use a slider to change sensitivity
        JLabel label1 = new JLabel("sensitivity=" + sensitivity);
        controls.add(label1);
        JSlider slider1 = new JSlider(1, 25, (int) (sensitivity * 100));
        slider1.setPreferredSize(new Dimension(50, 20));
        controls.add(slider1);
        slider1.addChangeListener(changeEvent -> {
            sensitivity = slider1.getValue() / 100.0;
            label1.setText("sensitivity=" + (int) (sensitivity * 100) / 100.0);
        });
        button = new Button("Corner Response");
        button.addActionListener(this);
        controls.add(button);
        JLabel label2 = new JLabel("threshold=" + threshold);
        controls.add(label2);
        JSlider slider2 = new JSlider(0, 100, threshold);
        slider2.setPreferredSize(new Dimension(50, 20));
        controls.add(slider2);
        slider2.addChangeListener(changeEvent -> {
            threshold = slider2.getValue();
            label2.setText("threshold=" + threshold);
        });
        button = new Button("Thresholding");
        button.addActionListener(this);
        controls.add(button);
        button = new Button("Non-max Suppression");
        button.addActionListener(this);
        controls.add(button);
        button = new Button("Display Corners");
        button.addActionListener(this);
        controls.add(button);
        // add two panels
        add("Center", main);
        add("South", controls);
        addWindowListener(new ExitListener());
        setSize(Math.max(width * 2 + 100, 850), height + 110);
        setVisible(true);
    }

    class ExitListener extends WindowAdapter {

        public void windowClosing(WindowEvent e) {
            System.exit(0);
        }
    }
    // Action listener for button click events

    public void actionPerformed(ActionEvent e) {
        // generate Moravec corner detection result
        if (((Button) e.getSource()).getLabel().equals("Derivatives")) {
            System.out.println("h");
        }
    }

    public static void main(String[] args) {
        new CornerDetection(args.length == 1 ? args[0] : "/home/adrian/NetBeansProjects/Assignment5 Sources/JavaApplication1/src/UFO.jpg");
    }

    private double gaussian(double x, double y, double sigma) {
        double sigma2 = sigma * sigma;
        double t = (x * x + y * y) / (2 * sigma2);
        double u = 1.0 / (2 * Math.PI * sigma2);
        double e = u * Math.exp(-t);
        return e;
    }


    /**
     * Compute the 3 arrays Ix, Iy and Ixy
     */
    private void computeDerivatives(double sigma) {

        Ix2 = new float[width][height];
        Iy2 = new float[width][height];
        Ixy = new float[width][height];
        int radius = (int) (2 * sigma);
        int window = 1 + 2 * radius;
        // gradient values: Gx,Gy
        float[][][] grad = new float[width][height][];
        for (int y = 0; y < width; y++) {
            for (int x = 0; x < width; x++) {
                grad[x][y] = sobel(x, y);
            }
        }
        // precompute the coefficients of the gaussian filter

        float[][] gaussian = new float[window][window];
        for (int j = -radius; j <= radius; j++) {
            for (int i = -radius; i <= radius; i++) {
                gaussian[i + radius][j + radius] = (float) gaussian(i, j, sigma);
            }
        }

        //
        for (int y = 0; y < width; y++) {
            for (int x = 0; x < width; x++) {

                for (int dy = -radius; dy <= radius; dy++) {
                    for (int dx = -radius; dx <= radius; dx++) {
                        int xk = x + dx;
                        int yk = y + dy;
                        if (xk < 0 || xk >= width) {
                            continue;
                        }
                        if (yk < 0 || yk >= height) {
                            continue;
                        }

                        // gaussian weight
                        double gw = gaussian[dx + radius][dy + radius];

                        // convolution
                        Ix2[x][y] += gw * grad[xk][yk][0] * grad[xk][yk][0];
                        Iy2[x][y] += gw * grad[xk][yk][1] * grad[xk][yk][1];
                        Ixy[x][y] += gw * grad[xk][yk][0] * grad[xk][yk][1];
                    }
                }
            }
        }
    }

    private float harrisMeasure(int x, int y, float k) {
        float m00 = Ix2[x][y];
        float m01 = Ixy[x][y];
        float m10 = Ixy[x][y];
        float m11 = Iy2[x][y];

        // Harris corner measure = det(M)-k.trace(M)^2
        return m00 * m11 - m01 * m10 - k * (m00 + m11) * (m00 + m11);
    }

    /**
     * return true if the measure at pixel (x,y) is a local spatial Maxima
     */
    private boolean isSpatialMaxima(float[][] hmap, int x, int y) {
        int n = 8;
        int[] dx = new int[]{-1, 0, 1, 1, 1, 0, -1, -1};
        int[] dy = new int[]{-1, -1, -1, 0, 1, 1, 1, 0};
        double w = hmap[x][y];
        for (int i = 0; i < n; i++) {
            double wk = hmap[x + dx[i]][y + dy[i]];
            if (wk >= w) {
                return false;
            }
        }
        return true;
    }

    
    
    private  float[] sobel(int x, int y) {
		int v00=0,v01=0,v02=0,v10=0,v12=0,v20=0,v21=0,v22=0;

		int x0 = x-1, x1 = x, x2 = x+1;
		int y0 = y-1, y1 = y, y2 = y+1;
		if (x0<0) x0=0;
		if (y0<0) y0=0;
		if (x2>=width) x2=width-1;
		if (y2>=height) y2=height-1;

		
		float sx = ((v20+2*v21+v22)-(v00+2*v01+v02))/(4*255f);
		float sy = ((v02+2*v12+v22)-(v00+2*v10+v20))/(4*255f);
		return new float[] {sx,sy};
	}
    
    
    
    /**
     * compute the Harris measure for each pixel of the image
     */
    private float[][] computeHarrisMap(double k) {
        return null;

    }

    public void filter(double sigma, double k, int minDistance) {

    }

}
