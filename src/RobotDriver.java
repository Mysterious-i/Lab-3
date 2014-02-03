import lejos.nxt.*;

public class RobotDriver {
	
	public static final double LEFT_RADIUS = 2.1;
	public static final double RIGHT_RADIUS = 2.1;		///// 2.85
	public static final double WIDTH = 15.45;
	public NXTRegulatedMotor leftMotor, rightMotor;
	private double forwardSpeed, rotationSpeed;
	
	/**
	 * Constructor
	 * 
	 * @param lMotor
	 *            The left motor of the robot.
	 * @param rMotor
	 *            The right motor of the robot.
	 */
	public RobotDriver(NXTRegulatedMotor leftMotor, NXTRegulatedMotor rightMotor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}
	
	 public double getDisplacement() {
	  		return (leftMotor.getTachoCount() * LEFT_RADIUS + rightMotor.getTachoCount() * RIGHT_RADIUS) * Math.PI / 360.0;
		}

		/**
		* return the robot's heading relative to its original orientation in degrees.
		*/
	   public double getHeading() {
		  return (-leftMotor.getTachoCount() * LEFT_RADIUS + rightMotor.getTachoCount() * RIGHT_RADIUS) / WIDTH;
		  ////// sign fixed
	   }
	   /**
		* Sets both the forward(cm/s) and rotation(deg/s) speeds of the robot..
		*/
	   public void setSpeeds(double forwardSpeed, double rotationSpeed) {
		  double leftSpeed, rightSpeed;

		  this.forwardSpeed = forwardSpeed;
		  this.rotationSpeed = rotationSpeed;

		  leftSpeed = (forwardSpeed - rotationSpeed * WIDTH * Math.PI / 360.0) * 180.0 / (LEFT_RADIUS * Math.PI);
		  rightSpeed = (forwardSpeed + rotationSpeed * WIDTH * Math.PI / 360.0) * 180.0 / (RIGHT_RADIUS * Math.PI);
		  // convert forwardspeed -> deg/sec and use the formulas on Navigation Tutorial

		  // set motor directions
		  if (leftSpeed > 0) {
		     leftMotor.forward();
		  } else {
		     leftMotor.backward();
		     leftSpeed = -leftSpeed;
		  }

		  if (rightSpeed > 0) {
		     rightMotor.forward();
		  } else {
		     rightMotor.backward();
		     rightSpeed = -rightSpeed;
		  }

		  // set motor speeds
		  if (leftSpeed > 900) {
		     leftMotor.setSpeed(900);
		  } else {
		     leftMotor.setSpeed((int)leftSpeed);
		  }

		  if (rightSpeed > 900) {
		     rightMotor.setSpeed(900);
		  } else {
		     rightMotor.setSpeed((int)rightSpeed);
		  }

	   }
	   
	   public void stop(){
		   leftMotor.setSpeed(0);
		   rightMotor.setSpeed(0);
	   }
}
 