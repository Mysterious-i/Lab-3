import lejos.nxt.*;

/**
 * Lab 3
 */

/**
 * @author Shahrzad Tighnavardmollasaraei 260413622
 *
 */
public class Lab3 {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO everything

		int buttonChoice;

		// some objects that need to be instantiated
		Odometer odometer = new Odometer();
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer);
		Navigation navigation = new Navigation(odometer);
		
		do {
			// clear the display
			LCD.clear();

			// ask the user whether the motors should drive in a square or float
			LCD.drawString("< Left | Right >", 0, 0);
			LCD.drawString("       |        ", 0, 1);
			LCD.drawString(" Float | Drive  ", 0, 2);
		
 
			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT
				&& buttonChoice != Button.ID_RIGHT);
		

		if (buttonChoice == Button.ID_LEFT) {
			for (NXTRegulatedMotor motor : new NXTRegulatedMotor[] { Motor.A, Motor.B, Motor.C }) {
				motor.forward();
				motor.flt();
			}

			// start only the odometer and the odometry display
			odometer.start();
			odometryDisplay.start();
		} else {
			// start the odometer, the odometry display and (possibly) the
			// odometry correction
			odometer.start();
			odometryDisplay.start();
			//odometryCorrection.start();
			navigation.start();

		}
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
		
		
	}
	

}
