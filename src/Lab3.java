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
		
		
		do {
			// clear the display
			LCD.clear();

			// ask the user whether the motors should drive in a square or float
			LCD.drawString("< Left | Right >", 0, 0);
			LCD.drawString("       |        ", 0, 1);
			LCD.drawString(" Path  | Path  	", 0, 2);
			LCD.drawString(" One   | Two    ", 0, 3);
 
			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT
				&& buttonChoice != Button.ID_RIGHT);
		

		if (buttonChoice == Button.ID_LEFT) 
		{	
			
			// start only the odometer and the odometry display
			odometer.start();
			odometryDisplay.start();
			Navigation navigation = new Navigation(odometer);
			navigation.start();
			
		} else {
			// start the odometer, the odometry display and (possibly) the
			// odometry correction
			odometer.start();
			odometryDisplay.start();
			//odometryCorrection.start();
			

		}
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
		
		
	}
	

}
