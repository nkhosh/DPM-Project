package tests;

import lejos.nxt.Button;
import lejos.nxt.Sound;
import main.*;

/**
 * This is the class used for the final demo run.
 * @author Niloofar Khoshsiyar
 *
 */
public class DemoTest {
	
	private static final double TILE_LENGTH = 30.48;
	
	private static Localizer loc;
	private static Navigator nav;
	private static Odometer odo;
	private static OdometryCorrector odoC;
	private static LCDPrinter lcd;
	private static int firstTargetX = 13, firstTargetY = 13, secondTargetX = 9, secondTargetY = 14;
	
	public static void main(String[] args) {
		RobinHood robin = new RobinHood();
		loc = robin.getLocalizer();
		nav = robin.getNavigator();
		odo = robin.getOdometer();
		odoC = robin.getOdometryCorrector();
		lcd = robin.getLCDPrinter();
		
		
		Button.waitForAnyPress();
		odo.start();
		lcd.start();
		
		// Localizing
		loc.doLocalization();
		Sound.beepSequence();
		odoC.start();
		
		// Avoiding obstacles and returning to start
		nav.travelToTiles(3.5, 2.5, true);
		nav.travelTo(8*TILE_LENGTH , 8*TILE_LENGTH , true);
		nav.travelTo(10*TILE_LENGTH , 10*TILE_LENGTH , false);
		
		// Localizing at the launching zone
		loc.doLocalization();
		odo.setPosition(new double[]{10*TILE_LENGTH,10*TILE_LENGTH,180});
		
		//Move to the appropriate position to fire at the targets
		nav.fireAtTiles(firstTargetX, firstTargetY, 8.5, 10.5, 8.5, 10.5, 3);
		nav.fireAtTiles(secondTargetX, secondTargetY, 8.25 ,10.4, 8.25, 10.4, 3);
		
		// Navigate back to the origin
		nav.travelTo(0 , 0 , true);
		loc.doLocalization();
		nav.travelTo(0,0, true);
		nav.turnToDeg(0, true);
		
		Button.waitForAnyPress();
		System.exit(0);

	}

}

