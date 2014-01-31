import lejos.nxt.Motor;

/*
 * Odometer.java
 */

public class Odometer extends Thread {
	// robot position
	private double x, y, theta;

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

	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;
	    int lastTachoL = 0;          /* Tacho L at last sample */
	    int lastTachoR = 0;          /* Tacho R at last sample */
	    int nowTachoL;           /* Current tacho L */
	    int nowTachoR;           /* Current tacho R */
	    double WR=2.1;           /* Wheel radius */
	    double WB=15.5;          /* Wheelbase */
		double distL, distR, deltaD, deltaT, dX, dY;
		while (true) {
			updateStart = System.currentTimeMillis();

			// put (some of) your odometer code here

			nowTachoL = Motor.A.getTachoCount();      
			nowTachoR = Motor.B.getTachoCount();
			distL = 3.14159*WR*(nowTachoL-lastTachoL)/180;
			distR = 3.14159*WR*(nowTachoR-lastTachoR)/180;
			lastTachoL=nowTachoL;
			lastTachoR=nowTachoR;
			deltaD = 0.5*(distL+distR);
			deltaT = (distL-distR)/WB;   
			
			synchronized (lock) {
			
             /* delta X, Y, Theta */
				theta += deltaT;
			    dX = deltaD * Math.sin(theta);
				dY = deltaD * Math.cos(theta);
				x = x + dX;                                      
				y = y + dY;
				
			}

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