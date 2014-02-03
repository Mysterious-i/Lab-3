import lejos.nxt.LCD;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.util.Timer;
import lejos.util.TimerListener;

public class UltrasonicPoller implements TimerListener{
	private UltrasonicSensor us;
	private Timer timer;
	public static final int US_PERIOD = 100;
	Odometer odometer;
	private Object lock = new Object(); 
	public static boolean obstacle = false;

	private int filterControl;
	private final int FILTER_OUT = 60;	

	public UltrasonicPoller() {
		this.us =  new UltrasonicSensor( SensorPort.S1);
		timer = new Timer(25, this);
		timer.start();
	}

	public void timedOut() {
		//process collected data
		double distance = us.getDistance();
		LCD.drawString("dist     "+ distance, 0, 6);


			synchronized (lock){
				if(distance<30){
					obstacle = true;
					LCD.drawString("Obstacle       ", 0, 5);
				}

				else{
					obstacle = false;
					LCD.drawString("NO Obstacle    ", 0, 5);
				}

			}

	}
}