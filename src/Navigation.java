
import lejos.nxt.*;
/**
 * @author Shahrzad Tighnavardmollasaraei 260413622
 * @author Alessandro Parisi 260413622	
 * Team 33
 */

public class Navigation extends Thread {
	//Robot Constants
	private final double WHEEL_RAD = 2.1;
	private final double WHEEL_BASE = 15.45;
	
	private boolean isTurning;
	private boolean isMoving;
	
	private Odometer odometer;

	private double deltaX;
	private double deltaY;
	private double deltaT;
	
	private double angleH; //the heading angle
	
	private double curX;
	private double curY;
	private double curT;
	
	
	private final NXTRegulatedMotor lMotor = Motor.A;
	private final NXTRegulatedMotor rMotor = Motor.B;
	double leftRadius, rightRadius, width;
	
	// Speed constants
	private static final int F_SPEED = 350; 
	private static final int R_SPEED = 150; 
	
	
	public void run(){
		//path one
		try{
			Thread.sleep(200);
		}catch (InterruptedException e){}
		travelTo(60,30);
		
		try{
			Thread.sleep(200);
		}catch (InterruptedException e){}
		travelTo(30,30);
		
		try{
			Thread.sleep(200);
		}catch (InterruptedException e){}
		travelTo(30,60);
		
		try{
			Thread.sleep(200);
		}catch (InterruptedException e){}
		travelTo(60,0);
	}
	//constructor
	public Navigation(Odometer od){
		this.odometer=od;
		isTurning = false;
		isMoving = false;
		
		// Reseting the motors
		for (NXTRegulatedMotor motor : new NXTRegulatedMotor[] { lMotor, rMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}
		
	}
	public void travelTo(double x, double y){
		isMoving = true;
		while (isMoving) { 
			//start the motors and moving
			lMotor.setSpeed(F_SPEED);
			rMotor.setSpeed(F_SPEED);
			lMotor.forward();
			rMotor.forward();
			
			curX = odometer.getX();
			curY = odometer.getY();
			
			double distance;
			deltaX = x - curX;
			deltaY = y - curY;
			
			// based on deltaY and deltaX here we figure out the actual angle the robot is heading  at
			if(deltaY>=0){
				angleH=Math.atan(deltaX/deltaY);
			}
			else if(deltaY<=0 && deltaX>=0){
				angleH = Math.atan(deltaX/deltaY) + Math.PI;
			}
			else{
				angleH = Math.atan(deltaX/deltaY) - Math.PI;
			}
		
			// turn to face the correct angle, then move forward

			turnTo(angleH* 180/ Math.PI);
			distance = Math.sqrt(deltaX*deltaX + deltaY*deltaY);
			
			lMotor.rotate(convertDistance(WHEEL_RAD, distance), true);
			rMotor.rotate(convertDistance(WHEEL_RAD, distance), false);
			
			//if we are close to the destination stop the wheels
			if(Math.abs(curX - x) <= .5 && Math.abs(curY - y) <= .5){
				this.isMoving = false;
				lMotor.stop(true);
				rMotor.stop();
				break;
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
	
	public boolean isNavigating(){
		
		return isTurning || isMoving;
		
	}
	 
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	//calculating the degree of turn
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}
