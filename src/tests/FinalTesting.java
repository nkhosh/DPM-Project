package tests;

import lejos.nxt.*;
import main.*;

/**
 * The class used for the integration testing.
 * It contains all the expected tasks of the robot.
 * Due to debugging, this class was subject to change and addition and deletion of many lines of code for different tests.
 * For the different versions of the test class please refer to Git.
 * @author Niloofar Khoshsiyar
 *
 */

public class FinalTesting {
	private static final double TILE_LENGTH = 30.48;
	public static void main (String[] args) {
		RobinHood robin = new RobinHood();
		Navigator navigator;
		double[][] demo = {{-.5*TILE_LENGTH,2*TILE_LENGTH},{-.5*TILE_LENGTH, 5.5*TILE_LENGTH},{1.5*TILE_LENGTH,5.5*TILE_LENGTH},{1.5*TILE_LENGTH,6.5*TILE_LENGTH},{4.5*TILE_LENGTH,6.5*TILE_LENGTH}};
		
		navigator = robin.getNavigator();
		robin.getOdometer().start();
		robin.getLCDPrinter().start();

		int buttonChoice = Button.waitForAnyPress();
		while(buttonChoice!=Button.ID_ENTER){
			buttonChoice = Button.waitForAnyPress();
		}
		robin.getLocalizer().doLocalization();
		Button.waitForAnyPress();

		robin.getOdometryCorrector().start();
		navigator.navigateMap(demo, true);
		Button.waitForAnyPress();
		
		navigator.turnToDeg(0,true);
		
		buttonChoice = Button.waitForAnyPress();
		while(buttonChoice!=Button.ID_ESCAPE) {
			robin.getOdometer().setX(0);
			robin.getOdometer().setY(0);
			robin.getOdometer().setHeading(0);
			buttonChoice = Button.waitForAnyPress();
		}
	}
}

