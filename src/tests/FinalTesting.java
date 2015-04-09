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
	
//	private static double[][] mapA = {
//		{0,7}
//	};
	
//	private static double[][] mapB = {
//		{0.5, 2.5},
//		{-2.5,2.5},
//		{-2.5,-0.5},
//		{0,0}
//	};
	
//	private static double[][] mapC = {
//		{0,4},
//		{9, 5}
//	};
	
//	private static double[][] integrationMap1 = {
//		{5*TILE_LENGTH - 7.5, 5*TILE_LENGTH -1 }
//	};
	
	private static double[][] integrationMap2 = {
		{0, 10}
	};
	
	private static double[][] calibrationMap = {
		{0,3},
		{1,1},
		{2,2},
		{0,0}
	};
	
	
	public static void main(String[] args) {
		RobinHood robin = new RobinHood();
		loc = robin.getLocalizer();
		nav = robin.getNavigator();
		odo = robin.getOdometer();
		odoC = robin.getOdometryCorrector();
		lcd = robin.getLCDPrinter();
//		LCDInfo lcdInfo = new LCDInfo(odo);
		
		
		Button.waitForAnyPress();
		odo.start();
//		lcd.start();
		
		// Localizing
//		loc.doLocalization();
		
		odoC.start();
//		
//		// Avoiding obstacles and returning to start
		nav.navigateMapTiles(calibrationMap, false);
//		nav.turnToDeg(0, true);
		Button.waitForAnyPress();
		
		odoC.setActive(false);
		loc.doLSLocalization(5*TILE_LENGTH, 5 * TILE_LENGTH);
		odoC.setActive(true);
//		
//		
		nav.fireAtTiles(9, 9, 3.5, 6.5, 3.5, 6.5, 6);
		
		Button.waitForAnyPress();
		nav.travelTo(0,0, true);
		nav.turnToDeg(0, true);
		
		Button.waitForAnyPress();
		System.exit(0);

	}

}
