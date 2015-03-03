import lejos.nxt.NXTRegulatedMotor;


public class Navigator {
	private Odometer odometer;
	private NXTRegulatedMotor rightWheel, leftWheel;
	public Navigator(Odometer odometer, NXTRegulatedMotor rightWheel, NXTRegulatedMotor leftWheel)	{
		this.odometer = odometer;
		this.rightWheel = rightWheel;
		this.leftWheel = leftWheel;
	}
}
