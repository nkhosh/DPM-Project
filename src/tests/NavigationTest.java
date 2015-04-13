package tests;

import lejos.nxt.Button;
import main.Navigator;
import main.RobinHood;

/**
 * Test class for different components of the navigation class, including travelTo, turnTo, fireAt
 * And obstacle avoidance.
 * This class was subject to change and addition and deletion of many lines of code for different tests.
 * For the different versions of the test class please refer to Git.
 * @author Niloofar Khoshsiyar
 *
 */
public class NavigationTest {
	public static void main (String[] args) {
		double TILE_LENGTH = 30.48;
		RobinHood robin = new RobinHood();
		Navigator navigator;

//		double[][] destination = {
//				{-TILE_LENGTH,-TILE_LENGTH},
//				{0,0},
//				{TILE_LENGTH,-TILE_LENGTH},
//				{0,0},
//				{TILE_LENGTH,TILE_LENGTH},
//				{0,0},
//				{-TILE_LENGTH,TILE_LENGTH},
//				{0,0}
//				};

		navigator = robin.getNavigator();

		int buttonChoice = Button.waitForAnyPress();
		while(buttonChoice!=Button.ID_ENTER && buttonChoice!=Button.ID_LEFT){
			buttonChoice = Button.waitForAnyPress();
		}
		if(buttonChoice==Button.ID_LEFT){
			robin.getOdometryCorrector().start();
		}
		robin.getOdometer().start();
		robin.getLCDPrinter().start();
		navigator.travelTo(2*TILE_LENGTH,2*TILE_LENGTH, true);
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
