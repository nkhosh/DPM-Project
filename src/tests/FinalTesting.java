package tests;

import lejos.nxt.Button;
import main.*;

public class FinalTesting {
	
	private static final double TILE_LENGTH = 30.48;
	
	private static Localizer loc;
	private static Navigator nav;
	private static Odometer odo;
	private static OdometryCorrector odoC;
	private static LCDPrinter lcd;
	
//	private static double[][] map = {
//			{2,2},
//			{2,5},
//			{0,7},
//			{0,0}
//			};
	
//	private static double[][] map = {
//		{2,2},
//		{4,0},
//		{6,0},
//		{4,2},
//		{0,0}
//		};
	
	
//	private static double[][] map = {
//		{1,2},
//		{2,1},
//		{0,0}
//		};
	
	private static double[][] map = {
		{0,6}
	};
	

	public static void main(String[] args) {
		RobinHood robin = new RobinHood();
		loc = robin.getLocalizer();
		nav = robin.getNavigator();
		odo = robin.getOdometer();
		odoC = robin.getOdometryCorrector();
		lcd = robin.getLCDPrinter();
		odo.start();
		
		Button.waitForAnyPress();
		lcd.start();
		
		// Localizing
//		loc.doLocalization();
		
		odoC.start();
//		
//		// Avoiding obstacles and returning to start
		nav.navigateMapTiles(map, true);
//		nav.turnToDeg(0, true);
//		nav.turnToDeg(120, true);
//		nav.turnToDeg(-90, true);
//		nav.turnToDeg(360, true);
//		nav.turnToDeg(180, true);
//		nav.turnToDeg(-180, true);
		
		Button.waitForAnyPress();
		System.exit(0);

	}

}
