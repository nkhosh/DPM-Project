package tests;

import lejos.nxt.Button;
import lejos.nxt.Sound;
import main.*;

public class FinalTesting {
	
	private static final double TILE_LENGTH = 30.48;
	
	private static Localizer loc;
	private static Navigator nav;
	private static Odometer odo;
	private static OdometryCorrector odoC;
	private static LCDPrinter lcd;
	private static int firstTargetX = 13, firstTargetY = 13, secondTargetX = 9, secondTargetY = 14;
	
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
	
//	private static double[][] integrationMap2 = {
//		{0, 10}
//	};
//	
//	private static double[][] calibrationMap = {
//		{0,3},
//		{1,1},
//		{2,2},
//		{0,0}
//	};
//	
//	
	
	private static double[][] map1 = {
		{1.5, 2.5},
		{1.5, 4.5},
		{-0.5, 4.5},
		{-0.5, 6.5},
		{0.5, 6.5},
		{2.5, 7.5},
		{2.5, 10.5},
		{0,0}
		
	};
	
	
	public static void main(String[] args) {
		RobinHood robin = new RobinHood();
		loc = robin.getLocalizer();
		nav = robin.getNavigator();
		odo = robin.getOdometer();
		odoC = robin.getOdometryCorrector();
		//lcd = robin.getLCDPrinter();
		LCDInfo lcdInfo = new LCDInfo(odo);
		
		
		Button.waitForAnyPress();
		odo.start();
//		lcd.start();
		
		// Localizing
		loc.doLocalization();
		Sound.beepSequence();
		//Button.waitForAnyPress();
//		odoC.start();
//		
//		// Avoiding obstacles and returning to start
//		nav.navigateMapTiles(calibrationMap, false);
		nav.travelToTiles(3.5, 2.5, true);
		nav.travelTo(8*TILE_LENGTH , 8*TILE_LENGTH , true);
		nav.travelTo(10*TILE_LENGTH , 10*TILE_LENGTH , false);
		
	//	Button.waitForAnyPress();
		
//		odoC.setActive(false);
		loc.doLocalization();
		odo.setPosition(new double[]{10*TILE_LENGTH,10*TILE_LENGTH,180});
//		odoC.setActive(true);
		
//		Button.waitForAnyPress();
//		
//		
		nav.fireAtTiles(firstTargetX, firstTargetY, 8.5, 10.5, 8.5, 10.5, 3);
		
		nav.fireAtTiles(secondTargetX, secondTargetY, 8.25 ,10.4, 8.25, 10.4, 3);
		
//		Button.waitForAnyPress();
		
		nav.travelTo(0 , 0 , true);
		//nav.turnToDeg(30, true);
	//	Button.waitForAnyPress();
		
//		odoC.setActive(false);
		loc.doLocalization();
//		odoC.setActive(true);
		
		
		
		nav.travelTo(0,0, true);
		nav.turnToDeg(0, true);
		
		Button.waitForAnyPress();
		System.exit(0);

	}

}
