package tests;

import lejos.nxt.Button;
import main.Navigator;
import main.RobinHood;

public class NavigationTest {
	public static void main (String[] args) {
		double TILE_LENGTH = 30.48;
		RobinHood robin = new RobinHood();
		Navigator navigator;
//		TouchSensor touchSensor = new TouchSensor(port);
		double[][] destination = {{0,30.48},{60.96,60.96},{0,0},{TILE_LENGTH,TILE_LENGTH}};
//		double[][] destination = {{60.96,60.96},{0,60.96},{30.48,30.48},{60.96,30.48},{0,0}};
//		double[][] destination = {{-30.48/2,60.96},{30.48/2,30.48/2},{60.96,60.96},{60.96,30.48/2},{0,0}};
//		double[][] destination = {{0,0}};
//		while(!touchSensor.isPressed());
//		double[][] demo1 = new double[5][2];
//		demo1[0] = new double[] {-.5*TILE_LENGTH,2*TILE_LENGTH};
//		demo1[1] = new double[] {-.5*TILE_LENGTH,5.5*TILE_LENGTH};
//		demo1[2] = new double[] {1.5*TILE_LENGTH,5.5*TILE_LENGTH};
//		demo1[3] = new double[] {1.5*TILE_LENGTH,6.5*TILE_LENGTH};
//		demo1[4] = new double[] {5*TILE_LENGTH,6.5*TILE_LENGTH};
		
		navigator = robin.getNavigator();

//		navigator.setDestinationArray(destination);

		int buttonChoice = Button.waitForAnyPress();
		while(buttonChoice!=Button.ID_ENTER && buttonChoice!=Button.ID_LEFT){
			buttonChoice = Button.waitForAnyPress();
		}
		if(buttonChoice==Button.ID_LEFT){
			robin.getOdometryCorrector().start();
		}
		robin.getOdometer().start();
		robin.getLcdPrinter().start();
//		navigator.turnTo(0,true);
//		for(int i=0;i<4;i++){
//			navigator.travelTo(destination[i][0], destination[i][1]);
//		}
		navigator.navigateMap(destination);
		navigator.turnTo(0, true);
		
		
//		navigator.travelTo(60.96,60.96, true);
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
