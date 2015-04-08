package main;
import lejos.nxt.ColorSensor;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;

public class RobinHood {
	private final static SensorPort lsPort = SensorPort.S1,
									usLeftPort = SensorPort.S2, usFrontPort = SensorPort.S3, usRightPort = SensorPort.S4; 
	private final static NXTRegulatedMotor[] WHEEL_MOTOR = {Motor.A, Motor.B};
	private final static NXTRegulatedMotor LAUNCHER_MOTOR = Motor.C;
	private static int numberOfPingPongBalls = 5;
	private final static int LEFT=0, RIGHT=1, FRONT=2;
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
		odometer = new Odometer(WHEEL_MOTOR, lock);
		odometryCorrector = new OdometryCorrector(odometer, lsController, lock);
		navigator = new Navigator(odometer, WHEEL_MOTOR, usController, launcher);
		launcher = new Launcher(LAUNCHER_MOTOR, numberOfPingPongBalls);
		lcdPrinter = new LCDPrinter(odometer, navigator, usController, lock);
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
