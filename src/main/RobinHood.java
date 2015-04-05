package main;
import lejos.nxt.ColorSensor;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;

public class RobinHood {
	private final static SensorPort lsPort = SensorPort.S1,
									usLeftPort = SensorPort.S2, usFrontPort = SensorPort.S3, usRightPort = SensorPort.S4; 
	private final static NXTRegulatedMotor[] wheelMotor = {Motor.A, Motor.B};
	private final static NXTRegulatedMotor launcherMotor = Motor.C;
	private final static int LEFT=0, RIGHT=1, FRONT=2;
	public final static double wheelRadius = 2.08, wheelsDistance = 16.09; // do we need this?
	private Object lock;
	private Odometer odometer;
	private OdometryCorrector odometryCorrector;
	private Navigator navigator;
	private Launcher launcher;
	private LCDPrinter lcdPrinter;
	private ColorSensor lightSensor;
	private UltrasonicSensor[] usSensors;
	private LSController lsController;
	private USController usController;
	private Localizer localizer;

	/**
	 * Constructor method. The variables are initialized here.
	*/
	public RobinHood() {
		usSensors = new UltrasonicSensor[3];
		usSensors[LEFT] = new UltrasonicSensor(usLeftPort);
		usSensors[RIGHT] = new UltrasonicSensor(usRightPort);
		usSensors[FRONT] = new UltrasonicSensor(usFrontPort);
		lightSensor = new ColorSensor(lsPort);
		
		lock = new Object();
		
		lsController = new LSController(lightSensor);
		usController = new USController(usSensors);
		odometer = new Odometer(wheelMotor, lock);
		odometryCorrector = new OdometryCorrector(odometer, lsController, lock);
		navigator = new Navigator(odometer, wheelMotor, usController, launcher);
		launcher = new Launcher(launcherMotor, 5);
		lcdPrinter = new LCDPrinter(odometer, navigator, lock);
		localizer = new Localizer(odometer,navigator,usSensors,lightSensor);
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
	
	public LCDPrinter getLcdPrinter() {
		return lcdPrinter;
	}
	
	public void run() {
		odometer.start();
	}

	public LSController getLScontroller() {
		// TODO Auto-generated method stub
		return lsController;
	}
}
