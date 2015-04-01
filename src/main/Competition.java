package main;
import lejos.nxt.*;


public class Competition {
		
//	@SuppressWarnings("unused")
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
		usSensor[0] = new UltrasonicSensor( SensorPort.S3 );
		usSensor[1] = new UltrasonicSensor(SensorPort.S4);
		lsSensor[0] = new ColorSensor(SensorPort.S1);
		lsSensor[1] = new ColorSensor(SensorPort.S2);
		Object lock = new Object();
		double TILE_LENGTH = 30.48;
		
		LSController lsController = new LSController(lsSensor);
		USController usController = new USController(usSensor);
		Odometer odometer = new Odometer(wheelMotor, lock);

		Launcher bow = new Launcher(odometer,Motor.C,5);
		//odometryCorrector = new OdometryCorrector(odometer, lsController, lock);
		//launcher = new Launcher(odometer, launcherMotor);
		LCDInfo lcdInfo = new LCDInfo(odometer);
		OdometryCorrector odoC = new OdometryCorrector(odometer,lsController,lock);
		Navigator navigator = new Navigator(odometer, wheelMotor, usController, bow, odoC);
		Localizer loc = new Localizer(odometer, navigator, usSensor,lsSensor);
		//LCDPrinter lcd = new LCDPrinter(odometer,lock);
		
		
		
		Button.waitForAnyPress();
		double[][] demo1 = new double[5][2];
		demo1[0] = new double[] {-.5*TILE_LENGTH,0};
		demo1[1] = new double[] {-.5*TILE_LENGTH,2*TILE_LENGTH};
		demo1[1] = new double[] {-.5*TILE_LENGTH,4*TILE_LENGTH};
		demo1[1] = new double[] {-.5*TILE_LENGTH,5.5*TILE_LENGTH};
		demo1[2] = new double[] {1.5*TILE_LENGTH,5.5*TILE_LENGTH};
		demo1[3] = new double[] {1.5*TILE_LENGTH,6.5*TILE_LENGTH};
		demo1[4] = new double[] {5*TILE_LENGTH,6.6*TILE_LENGTH};
		
		
		double[][] demo2 = {
				{5*TILE_LENGTH,6.5*TILE_LENGTH},
				{1.5*TILE_LENGTH,6.5*TILE_LENGTH},
				{1.5*TILE_LENGTH,5.5*TILE_LENGTH},
				{-.5*TILE_LENGTH,5.5*TILE_LENGTH},
				{-.5*TILE_LENGTH,2*TILE_LENGTH},
				{0-7.5,0-1}
		};
		//double[][]test = {{2*TILE_LENGTH,2*
		double[][] straight = new double[1][2];
		straight[0] = new double[] {-.35*TILE_LENGTH,5.6*TILE_LENGTH};
		//lcd.start();
		odometer.start();
		//lcdInfo.start();
		
		
		loc.doLocalization();
		Sound.beep();
		//Button.waitForAnyPress();
		//navigator.travelTo(TILE_LENGTH, TILE_LENGTH, false);
		odoC.start();
		navigator.navigateMap(demo1);
		navigator.travelTo(5*TILE_LENGTH -7.5, 5*TILE_LENGTH -1, false);
		navigator.turnTo(0, true);
//		Button.waitForAnyPress();
		odoC.setActive(false);
		loc.doLSLocalization(5*TILE_LENGTH,5*TILE_LENGTH);
	    //	Button.waitForAnyPress();
		odoC.setActive(true);
		navigator.fireAt(9*TILE_LENGTH, 9*TILE_LENGTH, 4.5*TILE_LENGTH,6.5*TILE_LENGTH,4.5*TILE_LENGTH,6.5*TILE_LENGTH, 6);
		navigator.navigateMap(demo2);
		odoC.setActive(false);
		//loc.doUSLocalization();
		loc.doLSLocalization(0,0);
		
//		navigator.travelTo(5.5*TILE_LENGTH, 5*TILE_LENGTH, true);
////		navigator.turnTo(0, true);
//		navigator.fireAt(9*TILE_LENGTH, 9*TILE_LENGTH, 4.5*TILE_LENGTH,6.5*TILE_LENGTH,4.5*TILE_LENGTH,6.5*TILE_LENGTH, 6);
//		navigator.travelTo(0, 0, true);
//		navigator.turnTo(0, true);
		
		
		
		

		Button.waitForAnyPress();
		System.exit(0);
		
		
	}

}


