import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.UltrasonicSensor;


public class Navigator {
	private Odometer odometer;
	private NXTRegulatedMotor rightWheel, leftWheel;
	
	public Navigator(Odometer odometer, NXTRegulatedMotor rightWheel, NXTRegulatedMotor leftWheel, UltrasonicSensor us1, UltrasonicSensor us2)	{
		this.odometer = odometer;
		this.rightWheel = rightWheel;
		this.leftWheel = leftWheel;
	}
	
	public void processUSData(UltrasonicSensor us) {
		
	}
}
