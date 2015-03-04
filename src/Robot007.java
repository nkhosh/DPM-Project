import lejos.nxt.ColorSensor;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.comm.Bluetooth;


public class Robot007 {
	private final static SensorPort lsPort1 = SensorPort.S1, lsPort2 = SensorPort.S2,
									usPort1 = SensorPort.S3, usPort2 = SensorPort.S4; //TODO: check the ports when connecting to hardware
	private final static NXTRegulatedMotor wheelMotorLeft = Motor.A, wheelMotorRight = Motor.B, launcherMotor = Motor.C;
	private Object lock;
	private Odometer odometer;
	private OdometryCorrector odometryCorrector;
	private Navigator navigator;
	private LSLocalizer lsLocalizer;
	private USLocalizer usLocalizer;
	private Launcher launcher;
	private LCDPrinter lcdPrinter;
	private ColorSensor lsSensor1, lsSensor2;
	private UltrasonicSensor usSensor1, usSensor2;

	/**
	 * Constructor method. The variables are initialized here.
	 */
	public Robot007() {
		usSensor1 = new UltrasonicSensor(usPort1);
		usSensor2 = new UltrasonicSensor(usPort2);
		lsSensor1 = new ColorSensor(lsPort1);
		lsSensor2 = new ColorSensor(lsPort2);
		lock = new Object();
		
		odometryCorrector = new OdometryCorrector(lock, lsSensor1, lsSensor2);
		odometer = new Odometer(odometryCorrector, lock);
		navigator = new Navigator(odometer,wheelMotorRight, wheelMotorLeft, usSensor1, usSensor2);
		lsLocalizer = new LSLocalizer(odometer, navigator, lsSensor1, lsSensor2);
		usLocalizer = new USLocalizer(odometer, navigator, usSensor1, usSensor2);
		launcher = new Launcher(odometer, launcherMotor);
		lcdPrinter = new LCDPrinter(odometer, lock);
	}
	
	/**
	 * Calibrates the light sensor using moving average method.
	 * @return true if the calibration is done successfully, false otherwise.
	 */
	public boolean calibrateLightSensor() {
		return true; //if the calibration is successful, false otherwise.
	}
	
	public int runMaster() {
		//TODO:
		//Localize
		//Navigate
		//OdometryCorrection
		return 0; //if successful, 1 otherwise
	}
	
	public int runSlave(double pitch) {
		//TODO:
		//Set pitch
		//Set shooting params
		//Shoot
		return 0; //if successful, 1 otherwise
	}
}
