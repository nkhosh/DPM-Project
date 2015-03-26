package tests;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.SensorPort;
import lejos.nxt.TouchSensor;
import main.Navigator;
import main.RobinHood;

public class NavigationTest {
	public static void main (String[] args) {
		RobinHood robin = new RobinHood();
		Navigator navigator;
		SensorPort port = SensorPort.S4;
		TouchSensor touchSensor = new TouchSensor(port);
//		double[][] destination = {{0,60.96},{60.96,60.96},{0,0},{30,30}};
//		double[][] destination = {{0,60.96},{30.48,30.48},{60.96,60.96},{60.96,0},{0,0}};
		double[][] destination = {{-30.48/2,60.96},{30.48/2,30.48/2},{60.96,60.96},{60.96,30.48/2},{0,0}};
		while(!touchSensor.isPressed());
	
		int buttonChoice;
		navigator = robin.getNavigator();
		robin.getOdometer().start();
//		robin.getOdometryCorrector().start();
		robin.getLcdPrinter().start();
		navigator.setDestinationArray(destination);
		navigator.start();
		navigator.turnTo(0,true);
		
		buttonChoice = Button.waitForAnyPress();
		while(buttonChoice!=Button.ID_ESCAPE) {
			robin.getOdometer().setX(0);
			robin.getOdometer().setY(0);
			robin.getOdometer().setHeading(0);
			navigator.setDestinationArray(destination);
			buttonChoice = Button.waitForAnyPress();
		}
	}
}
