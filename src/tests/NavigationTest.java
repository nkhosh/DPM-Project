package tests;

import lejos.nxt.Button;
import lejos.nxt.SensorPort;
import lejos.nxt.TouchSensor;
import main.Navigator;
import main.RobinHood;

public class NavigationTest {
	public static void main (String[] args) {
		final double TILE_LENGTH = 30.48;
		RobinHood robin = new RobinHood();
		Navigator navigator;
//		TouchSensor touchSensor = new TouchSensor(port);
//		double[][] destination = {{0,60.96},{60.96,60.96},{0,0},{30,30}};
//		double[][] destination = {{60.96,60.96},{0,60.96},{30.48,30.48},{60.96,30.48},{0,0}};
//		double[][] destination = {{-30.48/2,60.96},{30.48/2,30.48/2},{60.96,60.96},{60.96,30.48/2},{0,0}};
		double[][] destination = { 
				{-TILE_LENGTH/2 + 3, TILE_LENGTH*2.5},
				{-TILE_LENGTH/2 + 3, TILE_LENGTH*5.5},
				{TILE_LENGTH*1.5, TILE_LENGTH*5.5 + 4},
				{TILE_LENGTH*1.5, TILE_LENGTH*6.5},
				{TILE_LENGTH*4.5, TILE_LENGTH*6.5},
				{TILE_LENGTH*5.5, TILE_LENGTH*5.5},
				
		};
//		double[][] destination = { 
//				{0, TILE_LENGTH*3},
//		};
//		while(!touchSensor.isPressed());
		
		navigator = robin.getNavigator();
//		navigator.setDestinationArray(destination);

		int buttonChoice = Button.waitForAnyPress();
		while(buttonChoice!=Button.ID_ENTER){
			buttonChoice = Button.waitForAnyPress();
		}
//		navigator.turnTo(0,true);
//		for(int i=0;i<4;i++){
//			navigator.travelTo(destination[i][0], destination[i][1]);
//		}
		robin.getOdometer().start();
//		robin.getOdometryCorrector().start();
		robin.getLcdPrinter().start();
//		navigator.navigateMap(destination);
//		navigator.travelTo(60.96, 60.96, true);
		
		navigator.travelTo(60.96,60.96, true);
//		navigator.travelTo(0,0, true);
//		navigator.turnTo(0,true);
		
		buttonChoice = Button.waitForAnyPress();
		while(buttonChoice!=Button.ID_ESCAPE) {
			robin.getOdometer().setX(0);
			robin.getOdometer().setY(0);
			robin.getOdometer().setHeading(0);
//			navigator.setDestinationArray(destination);
			buttonChoice = Button.waitForAnyPress();
		}
	}
}
