package main;
import lejos.nxt.ColorSensor;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
/**
 * The main object assigned to the robot.
 * It connects the hardware to software by specifying the sensor and motor ports.
 * Then it initiates an instance of all the relevant objects in correct order
 * and passes them to the appropriate classes if necessary.
 * Since it holds one instance of each class and gives only reading access to other objects,
 * it acts like an interface to avoid multiple definition and initializations of same objects.
 * @author Niloofar Khoshsiyar
 *
 */
public class RobinHood {
	// sensor and motor ports constants
	private final static SensorPort LS_PORT = SensorPort.S1,
									US_LEFT_PORT = SensorPort.S2, US_FRONT_PORT = SensorPort.S3, US_RIGHT_PORT = SensorPort.S4; 
	private final static NXTRegulatedMotor[] WHEEL_MOTOR = {Motor.A, Motor.B};
	private final static NXTRegulatedMotor LAUNCHER_MOTOR = Motor.C;
	
	// Useful flags
	private final static int LEFT=0, RIGHT=1, FRONT=2;
	
	// Class objects
	private Object lock;
	private Odometer odometer;
	private OdometryCorrector odometryCorrector;
	private Navigator navigator;
	private LCDPrinter lcdPrinter;
	private ColorSensor lightSensor;
	private UltrasonicSensor[] usSensors;
	private LSController lsController;
	private USController usController;
	private Localizer localizer;
	private Launcher launcher;
	private static int numberOfPingPongBalls = 5;

	/**
	 * Constructor method. The variables are initialized here.
	*/
	public RobinHood() {
		usSensors = new UltrasonicSensor[3];
		usSensors[LEFT] = new UltrasonicSensor(US_LEFT_PORT);
		usSensors[RIGHT] = new UltrasonicSensor(US_RIGHT_PORT);
		usSensors[FRONT] = new UltrasonicSensor(US_FRONT_PORT);
		lightSensor = new ColorSensor(LS_PORT);
		
		lock = new Object();
		
		lsController = new LSController(lightSensor);
		usController = new USController(usSensors);
		odometer = new Odometer(WHEEL_MOTOR, lock);
		odometryCorrector = new OdometryCorrector(odometer, lsController, lock);
		launcher = new Launcher(LAUNCHER_MOTOR, numberOfPingPongBalls);
		navigator = new Navigator(odometer, WHEEL_MOTOR, usController, launcher);
		lcdPrinter = new LCDPrinter(odometer, navigator, usController, lock);
		localizer = new Localizer(odometer,navigator,usController,lsController);
	}
	
	public Odometer getOdometer() {
		return odometer;
	}
	
	public Localizer getLocalizer() {
		return localizer;
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
	
	public LCDPrinter getLCDPrinter() {
		return lcdPrinter;
	}
	
	public void run() {
		odometer.start();
	}

	public LSController getLScontroller() {
		return lsController;
	}
}
