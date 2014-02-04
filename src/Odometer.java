import lejos.nxt.Motor;

/*
 * Odometer.java
 *  
 */

/**
 * @author Shahrzad Tighnavardmollasaraei 260413622
 * @author Alessandro Parisi 260413622	
 * Team 33
 */

public class Odometer extends Thread {
	// robot position
	private double x, y, theta;

	/**
	 * Class Constants
	 */
		private static final double WHEEL_BASE = 15.45; //Base of the wheels
		private static final double WHEEL_RAD = 2.1; //Radius of the wheels
	
	// odometer update period, in ms
	private static final long ODOMETER_PERIOD = 25;

	// lock object for mutual exclusion
	private Object lock;

	// default constructor
	public Odometer() {
		x = 0.0;
		y = 0.0;
		theta = 0.0;
		lock = new Object();
	}

	/**
	 * Class Variables
	 */
	private static int lastTachoL = 0 ; /* Tacho L at last sample */
	private static int lastTachoR = 0; /* Tacho R at last sample */

	private static int presTachoL; /* present tacho L*/
	private static int presTachoR; /* present tacho R */
	
	
	private static double deltaTR; 
	private static double deltaTL;
	
	private double avgD;
	private double deltaDL;
	private double deltaDR;
	private double deltaT;
	
	
	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;
	
	    double dX, dY;
		
	    while (true) {
			updateStart = System.currentTimeMillis();

			

			presTachoL = Motor.A.getTachoCount();      
			presTachoR = Motor.B.getTachoCount();
			
			deltaTL = presTachoL-lastTachoL;
			deltaTR = presTachoR-lastTachoR;
			
			deltaDL = 3.14159*WHEEL_RAD*(deltaTL)/180;
			deltaDR = 3.14159*WHEEL_RAD*(deltaTR)/180;
			
			
			
			avgD = 0.5*(deltaDL+deltaDR);
			
			deltaT = (deltaDL-deltaDR)/WHEEL_BASE;   
			
			synchronized (lock) {
			
             /* delta X, Y, Theta */
				theta += deltaT;
			    dX = avgD * Math.sin(theta);
				dY = avgD * Math.cos(theta);
				x = x + dX;                                      
				y = y + dY;
				
			}
			
			
			lastTachoL=presTachoL;
			lastTachoR=presTachoR;
			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}

	// accessors
	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = theta;
		}
	}

	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}
	

	// mutators
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}
}