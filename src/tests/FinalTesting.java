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
	
	private static double[][] integrationMap1 = {
		{0 - 7.5, 10*TILE_LENGTH - 1}
	};
	public static void main(String[] args) {
		RobinHood robin = new RobinHood();
		loc = robin.getLocalizer();
		nav = robin.getNavigator();
		odo = robin.getOdometer();
		odoC = robin.getOdometryCorrector();
		lcd = robin.getLCDPrinter();
		LCDInfo lcdInfo = new LCDInfo(odo);
		odo.start();
		
		Button.waitForAnyPress();
//		lcd.start();
		
		// Localizing
		loc.doLocalization();
		
		odoC.start();
//		
//		// Avoiding obstacles and returning to start
		nav.navigateMap(integrationMap1, true);
		
		odoC.setActive(false);
		loc.doLSLocalization(0, 10 * TILE_LENGTH);
		odoC.setActive(true);
		
		
		nav.fireAtTiles(4, 9, -0.5, 1.5, 8.5, 10.5, 3);
		nav.travelTo(0,0, true);
		
		Button.waitForAnyPress();
		System.exit(0);

	}

}
