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
	double sensitivity=.1;
	int threshold=20;
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
		}
		catch ( Exception ex ) {
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
		JSlider slider1 = new JSlider(1, 25, (int)(sensitivity*100));
		slider1.setPreferredSize(new Dimension(50, 20));
		controls.add(slider1);
		slider1.addChangeListener(changeEvent -> {
			sensitivity = slider1.getValue() / 100.0;
			label1.setText("sensitivity=" + (int)(sensitivity*100)/100.0);
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
		setSize(Math.max(width*2+100,850), height+110);
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
		if ( ((Button)e.getSource()).getLabel().equals("Derivatives") )
			System.out.println("h");
	}
	public static void main(String[] args) {
		new CornerDetection(args.length==1 ? args[0] : "/home/adrian/NetBeansProjects/Assignment5 Sources/JavaApplication1/src/UFO.jpg");
	}

	// moravec implementation
	void derivatives() {
		int l, t, r, b, dx, dy;
		Color clr1, clr2;
		int gray1, gray2;
                
                this.Ix2 = new float[width][height];
		this.Iy2 = new float[width][height];
		this.Ixy = new float[height][height];
                
                
		for ( int q=0 ; q<height ; q++ ) {
			t = q==0 ? q : q-1;
			b = q==height-1 ? q : q+1;
			for ( int p=0 ; p<width ; p++ ) {
				l = p==0 ? p : p-1;
				r = p==width-1 ? p : p+1;
				clr1 = new Color(source.image.getRGB(l,q));
				clr2 = new Color(source.image.getRGB(r,q));
				gray1 = clr1.getRed() + clr1.getGreen() + clr1.getBlue();
				gray2 = clr2.getRed() + clr2.getGreen() + clr2.getBlue();
				dx = (gray2 - gray1) / 3;
				clr1 = new Color(source.image.getRGB(p,t));
				clr2 = new Color(source.image.getRGB(p,b));
				gray1 = clr1.getRed() + clr1.getGreen() + clr1.getBlue();
				gray2 = clr2.getRed() + clr2.getGreen() + clr2.getBlue();
				dy = (gray2 - gray1) / 3;
				dx = Math.max(-128, Math.min(dx, 127));
				dy = Math.max(-128, Math.min(dy, 127));
				target.image.setRGB(p, q, new Color(dx+128, dy+128, 128).getRGB());
			}
		}
		target.repaint();
	}
        private  double gaussian(double x, double y, double sigma) {
		double sigma2 = sigma*sigma;
		double t = (x*x+y*y)/(2*sigma2);
		double u = 1.0/(2*Math.PI*sigma2);
		double e = u*Math.exp( -t );
		return e;
	}

        
        
        
}