package main;
import lejos.nxt.*;


public class Competition {
	

	
	@SuppressWarnings("unused")
	public static void main(String[] args) {
		// setup the odometer, display, and ultrasonic and light sensors
		//Button.waitForAnyPress();
	/*	
		private final static SensorPort lsLeftPort = SensorPort.S1, lsRightPort = SensorPort.S2,
				usLeftPort = SensorPort.S3, usFrontPort = SensorPort.S4; //TODO: check the ports when connecting to hardware
	private final static NXTRegulatedMotor[] wheelMotor = {Motor.A, Motor.B};
	private final static NXTRegulatedMotor launcherMotor = Motor.C;
	public final static double wheelRadius = 2.08, wheelsDistance = 16.09; // do we need this?
	private Object lock;
	private Odometer odometer;
	//private OdometryCorrector odometryCorrector;
	private Navigator navigator;
	//private Launcher launcher;
	private LCDPrinter lcdPrinter;
	private ColorSensor[] lsSensor;
	private UltrasonicSensor[] usSensor;
	private LSController lsController;
	private USController usController;
	*/

		NXTRegulatedMotor[] wheelMotor = {Motor.A, Motor.B};
		UltrasonicSensor[] usSensor = new UltrasonicSensor[2];
		ColorSensor[] lsSensor = new ColorSensor[2];
		usSensor[0] = new UltrasonicSensor( SensorPort.S4 );
		usSensor[1] = new UltrasonicSensor(SensorPort.S3);
		lsSensor[0] = new ColorSensor(SensorPort.S1);
		lsSensor[1] = new ColorSensor(SensorPort.S2);
		Object lock = new Object();
		
		LSController lsController = new LSController(lsSensor);
		USController usController = new USController(usSensor);
		Odometer odometer = new Odometer(wheelMotor, lock);

		Launcher bow = new Launcher(odometer,Motor.C,4);
		//odometryCorrector = new OdometryCorrector(odometer, lsController, lock);
		Navigator navigator = new Navigator(odometer, wheelMotor, usController,bow);
		Localizer loc = new Localizer(odometer, navigator, usSensor,lsSensor);
		//launcher = new Launcher(odometer, launcherMotor);
		LCDInfo lcdInfo = new LCDInfo(odometer);
		OdometryCorrector odoC = new OdometryCorrector(odometer,lsController,lock);
		
		
		Button.waitForAnyPress();
		double[][] demo = new double[5][2];
		demo[0] = new double[] {-.35*30.48,2*30.48};
		demo[1] = new double[] {-.35*30.48,5.6*30.48};
		demo[2] = new double[] {1.5*30.48,5.6*30.48};
		demo[3] = new double[] {1.5*30.48,6.6*30.48};
		demo[4] = new double[] {5*30.48,6.6*30.48};
		
		double[][] straight = new double[1][2];
		straight[0] = new double[] {0,6*30.48};
		odometer.start();
		//lcdInfo.start();
		//
		loc.doLocalization();
		//Button.waitForAnyPress();
		navigator.travelTo(30.48, 30.48, false);
		odoC.start();
		//navigator.navigateMap(demo);
		navigator.fireAt(5*30.48, 5*30.48, 20.0,3*30.48 - 20.0, 20.0, 3*30.48 - 20, 3);
		
		
		

		Button.waitForAnyPress();
		System.exit(0);
		
		
	}

}


