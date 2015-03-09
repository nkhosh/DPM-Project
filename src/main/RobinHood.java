package main;
import lejos.nxt.ColorSensor;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;

public class RobinHood {
	private final static SensorPort lsPort1 = SensorPort.S1, lsPort2 = SensorPort.S2,
									usPort1 = SensorPort.S3, usPort2 = SensorPort.S4; //TODO: check the ports when connecting to hardware
	private final static NXTRegulatedMotor[] wheelMotor = {Motor.A, Motor.B};
	private final static NXTRegulatedMotor launcherMotor = Motor.C;
	private Object lock;
	private Odometer odometer;
	private OdometryCorrector odometryCorrector;
	private Navigator navigator;
	private Launcher launcher;
	private LCDPrinter lcdPrinter;
	private ColorSensor lsSensor[];
	private UltrasonicSensor usSensor[];
	private LSController lsController;
	private USController usController;

/**
 * Constructor method. The variables are initialized here.
*/
	public RobinHood() {
		usSensor = new UltrasonicSensor[2];
		lsSensor = new ColorSensor[2];
		usSensor[0] = new UltrasonicSensor(usPort1);
		usSensor[1] = new UltrasonicSensor(usPort2);
		lsSensor[0] = new ColorSensor(lsPort1);
		lsSensor[1] = new ColorSensor(lsPort2);
		lock = new Object();
		
		lsController = new LSController(lsSensor);
		usController = new USController(usSensor);
		odometryCorrector = new OdometryCorrector(odometer, lsController, lock);
		odometer = new Odometer(odometryCorrector, lock);
		navigator = new Navigator(odometer, wheelMotor, usController);
		launcher = new Launcher(odometer, launcherMotor);
		lcdPrinter = new LCDPrinter(odometer, lock);
	}
	
	public Odometer getOdometer() {
	return odometer;
	}
	
	public OdometryCorrector getOdometryCorrector() {
		return odometryCorrector;
	}
	
	public Navigator getNavigator() {
		return navigator;
	}
	
	public Launcher getLauncher() {
		return launcher;
	}
	
	public LCDPrinter getLcdPrinter() {
		return lcdPrinter;
	}

	/**
	 * Calibrates the light sensor using moving average method.
	 * @return true if the calibration is done successfully, false otherwise.
	 */
	public boolean calibrateLightSensor() {
		return true; //if the calibration is successful, false otherwise.
	}
	
	public void run() {
		odometer.start();
	}
}
