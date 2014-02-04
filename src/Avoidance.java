import lejos.nxt.*;


/**
 * @author Shahrzad Tighnavardmollasaraei 260413622
 * @author Alessandro Parisi 260413622	
 * Team 33
 */
public class Avoidance extends Thread{
	//Boolean variables for navigation
	private boolean isMoving;
	private boolean obstacleDetected;
	
	//Required Constants
	private static final int F_SPEED = 250, C_SPEED = 175; 
	private static final int R_SPEED = 150; 
	private final int BC = 10;//band center
	private final int BW = 2;//bandwidth
	private final double WHEEL_RAD = 2.1, WHEEL_BASE = 15.45;
	private final int THRESHOLD = 10;
	
	
	//Coordinate variables
	private double curX = 0;
	private double curY = 0;
	private double curT = 0;
	private double deltaX; 
	private double deltaY; 
	private double deltaT;
	private double theta; //heading direction
	//ultrasonic Sensor's reading value 
	int sReading;
	
	//Variables for the motors and the wheels.
	private final NXTRegulatedMotor lMotor = Motor.A;//left 
	private final NXTRegulatedMotor rMotor = Motor.B;//right
	private final NXTRegulatedMotor sMotor = Motor.C;//sensor
	
	
	
	
	//Variables for the PController
	private int filterControl;
	private final int FILTER_ERROR = 18; //ultrasonic error readings to be filtered 
	private int correction = 0; //speed correction
	private int error=0;
	
	//Odometer and Ultrasonic Sensor.
	private Odometer odometer;
	private UltrasonicSensor sonicS = new UltrasonicSensor(SensorPort.S1);
	private boolean isTurning;
	
	public Avoidance (Odometer odometer){ //Constructor
		this.odometer = odometer; 
		
		// reset the motors
		for (NXTRegulatedMotor motor : new NXTRegulatedMotor[] { lMotor, rMotor, sMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}
	}
	public void run(){ // executes program
	//path two
		try{
			Thread.sleep(200);
		} catch (InterruptedException e){
			
		}
		
		travelTo(0,60);
		try{
			Thread.sleep(200);
		} catch (InterruptedException e){
			
		}

		
		travelTo(60,0);
		try{
			Thread.sleep(200);
		} catch (InterruptedException e){
		
		}
		
	}
	
	public void travelTo (double x, double y){ //Takes the target coordinates when called upon
		this.isMoving = true; //the robot is moving
		while (isMoving){
			//start the motors and moving
			lMotor.setSpeed(F_SPEED);
			rMotor.setSpeed(F_SPEED);
			lMotor.forward();
			rMotor.forward();
			
			//set up the coordinate variables 
			curX = odometer.getX();
			curY = odometer.getY();
			deltaX = x - curX;
			deltaY = y - curY;
			
			if(deltaY>=0){
				theta=Math.atan(deltaX/deltaY);
			}
			else if(deltaY<=0 && deltaX>=0){
				theta = Math.atan(deltaX/deltaY) + Math.PI;
			}
			else{
				theta = Math.atan(deltaX/deltaY) - Math.PI;
			}
			
			//If there's an obstacle within the threshold distance , try and avoid it. 
			if (sonicS.getDistance() <= THRESHOLD){
				
				avoidObstacle(x,y);
			}
			
			
			// turn to face the correct angle, then move forward

			turnTo(theta* 180/ Math.PI);
			double distance = Math.sqrt(deltaX*deltaX + deltaY*deltaY);
						
			lMotor.rotate(convertDistance(WHEEL_RAD, distance), true);
			rMotor.rotate(convertDistance(WHEEL_RAD, distance), false);
			
			
			//Checks the robot's current position. If it finishes within 5mm of target coordinate, the robot ceases its algorithm. 
			if(Math.abs(curX - x) <= .5 && Math.abs(curY - y) <= .5){
				this.isMoving = false;
				lMotor.stop(true);
				rMotor.stop();
			}
			
		}
		
	}
	public void turnTo(double theta){
		isTurning = true;
		//the current heading
		curT = odometer.getTheta()* 180/ Math.PI;
		deltaT = theta - curT ;
		// whether we need to make a left turn or right
		if(deltaT > 180){
			deltaT = deltaT - 360;
		}
		else if(deltaT<=-180){
			deltaT = deltaT + 360;
		}
		LCD.drawString(""+deltaT, 0, 5);
		
		lMotor.setSpeed(R_SPEED);
		rMotor.setSpeed(R_SPEED);
		
		lMotor.rotate(convertAngle(WHEEL_RAD, WHEEL_BASE, deltaT), true);
		rMotor.rotate(-convertAngle(WHEEL_RAD, WHEEL_BASE, deltaT), false);
		
		isTurning = false;
		
	}
	
	public void avoidObstacle(double x, double y){
		
		//stop moving 
		lMotor.stop(true); //synchronize both wheels.
		rMotor.stop();
		sMotor.rotate(-50); //make the sensor face the wall
		//going to circle around the obstacle
		turnTo(odometer.getTheta() + 90); //Aligns the robot to the wall.
		
		obstacleDetected = true;
		
		//Start orbiting around.
		lMotor.setSpeed(F_SPEED);
		rMotor.setSpeed(F_SPEED);
		lMotor.forward();
		rMotor.forward();
		
		//if the obstacle is no longer on robots path
		while(obstacleDetected)
		{
			//setting up the coordinate variables
			curX = odometer.getX();
			curY = odometer.getY();
			deltaX = x - curX;
			deltaY = y - curY;
			curT = odometer.getTheta();
			
			
			
			if(deltaY>=0){
				theta=Math.atan(deltaX/deltaY);
			}
			else if(deltaY<=0 && deltaX>=0){
				theta = Math.atan(deltaX/deltaY) + Math.PI;
			}
			else{
				theta = Math.atan(deltaX/deltaY) - Math.PI;
			}
			
			
			//making sure the angles are within the range [-pi,pi]
			if (theta > 180){
				theta -= 360;
			} else if (theta <= -180){
				theta += 360;
			}
			if (curT > 180){
				curT -= 360;
			} else if (curT <= -180){
				curT += 360;
			}
			
			//Difference of the current heading and the required heading angle
			double deltaAngle = curT - theta;
			if (deltaAngle > 180){
				deltaAngle -= 360;
			} else if (deltaAngle <= -180){
				deltaAngle += 360;
			}
			deltaAngle = Math.abs(deltaAngle);
			
			//If the robot is heading in the correct direction (destination)
			if (deltaAngle <= 2){
				
				obstacleDetected = false;
				sMotor.rotate(50, true); //Brings sensor back facing forward for the next obstacle.
				
			}
			
			//Calling the pController for avoiding the obstacle
			pControl(sonicS.getDistance()); 
		}
		
	}
	public void pControl(int distance){//PController from lab 1
		// rudimentary filter
		if (distance == 255 && filterControl < FILTER_ERROR) {
			// bad value, do not set the distance var, however do increment the filter value
			filterControl ++;
		} else if (distance == 255){
			// true 255, therefore set distance to 255
			this.sReading = distance;
		} else {
			// distance went below 255, therefore reset everything.
			filterControl = 0;
			this.sReading = distance;
		}
		
		/* the following lines of code modifies the speed of the motors depending :
		 * - if the absolute value of the error is within the accpeted value 
		 * - or if the error is -ive ( too close to the wall),
		 * - or if the error is +ive ( too far from the wall ).
		 */
		calcProp();
		
		error = this.BC - this.sReading;
		if(Math.abs(error) <= BW){
			lMotor.setSpeed(C_SPEED);
			rMotor.setSpeed(C_SPEED);
			
		}
		else if (error < 0){ // turn left
			rMotor.setSpeed(C_SPEED + correction);
			lMotor.setSpeed(C_SPEED - correction);
		}
		
		else if(error > 0){ //turn right
			lMotor.setSpeed(C_SPEED + correction);
			rMotor.setSpeed(C_SPEED - correction); 
			
		}
	}
	
	
	//
	public boolean isNavigating(){
		return isMoving || isTurning ;
	}
	
	//helper methods
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	//Converts to degrees of turns it needs based on the wheels' radius and the angle it needs to rotate.
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	/*
	 *	This method calculates the the correction( int ) which is then
	 * 	 used to increase or decrease the speed of the motors
	 */
	public void calcProp(){
		int absError=Math.abs(error);
		correction= 4 * absError;
		
		if(absError > 25){
			correction = 100; 
		}
				
	}

	
}
