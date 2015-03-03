import lejos.nxt.ColorSensor;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;


public class Robot007 {
	private final static SensorPort lsPort = SensorPort.S1, usPort = SensorPort.S2; //TODO: check the ports when connecting to hardware
	private final static NXTRegulatedMotor wheelMotorLeft = Motor.A, wheelMotorRight = Motor.B, launcherMotor = Motor.C;
	private Object lock;
	
	private Odometer odometer;
	private Navigator navigator;
	private LSLocalizer lsLocalizer;
	private USLocalizer usLocalizer;
	private Launcher launcher;
	private LCDPrinter lcdPrinter;
	private ColorSensor lsSensor;
	private UltrasonicSensor usSensor;

	
	public Robot007() {
		usSensor = new UltrasonicSensor(usPort);
		lsSensor = new ColorSensor(lsPort);
		lock = new Object();
		
		odometer = new Odometer(lock);
		navigator = new Navigator(odometer,wheelMotorRight, wheelMotorLeft);
		lsLocalizer = new LSLocalizer(odometer, navigator, lsSensor);
		usLocalizer = new USLocalizer(odometer, navigator, usSensor);
		launcher = new Launcher(odometer, launcherMotor);
		lcdPrinter = new LCDPrinter(odometer, lock);
	}
	
	public boolean calibrateLightSensor() {
		return true; //if the calibration is successful, false otherwise.
	}
}
