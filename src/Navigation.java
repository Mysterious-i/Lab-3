import lejos.nxt.LCD;
import lejos.nxt.Motor;


public class Navigation extends Thread {
	
	private final double WHEEL_RAD = 2.1;
	private final double WIDTH = 15.45;
	private boolean isTurning;
	private boolean isMoving;
	private Odometer odometer;

	public void run(){
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
	
	public Navigation(Odometer od){
		this.odometer=od;
		isTurning = false;
		isMoving = false;
	}
	public void travelTo(double x, double y){
		isMoving = true;
		double distance;
		double diffX = x - odometer.getX();
		double diffY = y - odometer.getY();
		double tAngle;
		if(diffY>=0){
			tAngle=Math.atan(diffX/diffY);
		}
		else if(diffY<=0 && diffX>=0){
			tAngle = Math.atan(diffX/diffY) + Math.PI;
		}
		else{
			tAngle = Math.atan(diffX/diffY) - Math.PI;
		}
	
		
		turnTo(tAngle* 180/ Math.PI);
		distance = Math.sqrt(diffX*diffX + diffY*diffY);
		
		Motor.A.rotate(convertDistance(WHEEL_RAD, distance), true);
		Motor.B.rotate(convertDistance(WHEEL_RAD, distance), false);
		Motor.A.stop();
		Motor.B.stop();  
		isMoving = false;
		
		
	}
	public void turnTo(double theta){
		isTurning = true;
		double angle = theta - odometer.getTheta()* 180/ Math.PI;
		
		if(angle > 180){
			angle = angle - 360;
		}
		else if(angle<-180){
			angle = angle + 360;
		}
		LCD.drawString(""+angle, 0, 5);
		//odometer.setTheta(angle);
		Motor.A.setSpeed(200);
		Motor.B.setSpeed(200);
		
		Motor.A.rotate(convertAngle(WHEEL_RAD, WIDTH, angle), true);
		Motor.B.rotate(-convertAngle(WHEEL_RAD, WIDTH, angle), false);
		isTurning = false;
		
		
	}
	
	public boolean isNavigating(){
		
		return isTurning || isMoving;
		
	}
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}
