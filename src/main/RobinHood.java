package main;
import lejos.nxt.ColorSensor;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;

public class RobinHood {
	private final static SensorPort lsLeftPort = SensorPort.S1, lsRightPort = SensorPort.S2,
									usLeftPort = SensorPort.S3, usFrontPort = SensorPort.S4; //TODO: check the ports when connecting to hardware
	private final static NXTRegulatedMotor[] wheelMotor = {Motor.A, Motor.B};
	private final static NXTRegulatedMotor launcherMotor = Motor.C;
	public final static double wheelRadius = 2.08, wheelsDistance = 16.09; // do we need this?
	private Object lock;
	private Odometer odometer;
	private OdometryCorrector odometryCorrector;
	private Navigator navigator;
	private Launcher launcher;
	private LCDPrinter lcdPrinter;
	private ColorSensor[] lsSensor;
	private UltrasonicSensor[] usSensor;
	private LSController lsController;
	private USController usController;
	private Localizer localizer;

	/**
	 * Constructor method. The variables are initialized here.
	*/
	public RobinHood() {
		usSensor = new UltrasonicSensor[2];
		lsSensor = new ColorSensor[2];
		usSensor[0] = new UltrasonicSensor(usFrontPort);
		usSensor[1] = new UltrasonicSensor(usLeftPort);
		lsSensor[0] = new ColorSensor(lsLeftPort);
		lsSensor[1] = new ColorSensor(lsRightPort);
		
		lock = new Object();
		
		lsController = new LSController(lsSensor);
		usController = new USController(usSensor);
		odometer = new Odometer(wheelMotor, lock);
		odometryCorrector = new OdometryCorrector(odometer, lsController, lock);
		launcher = new Launcher(odometer, launcherMotor, 6);
		navigator = new Navigator(odometer, wheelMotor, usController, launcher);
		lcdPrinter = new LCDPrinter(odometer, lock);
		localizer = new Localizer(odometer,navigator,usSensor,lsSensor);
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
