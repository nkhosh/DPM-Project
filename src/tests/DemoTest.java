package tests;

import lejos.nxt.*;
import main.*;

public class DemoTest {
	public static void main (String[] args) {
		RobinHood robin = new RobinHood();
		Navigator navigator;
		SensorPort port = SensorPort.S4;
		double segment = 30.48;
//		TouchSensor touchSensor = new TouchSensor(port);
//		double[][] destination = {{0,60.96},{60.96,60.96},{0,0},{30,30}};
		double[][] demo = {{-.5*segment,2*segment},{-.5*segment, 5.5*segment},{1.5*segment,5.5*segment},{1.5*segment,6.5*segment},{4.5*segment,6.5*segment}};
//		double[][] destination = {{-30.48/2,60.96},{30.48/2,30.48/2},{60.96,60.96},{60.96,30.48/2},{0,0}};
//		while(!touchSensor.isPressed());
		
		navigator = robin.getNavigator();
		robin.getOdometer().start();
		robin.getLCDPrinter().start();
//		navigator.setDestinationArray(destination);

		int buttonChoice = Button.waitForAnyPress();
		while(buttonChoice!=Button.ID_ENTER){
			buttonChoice = Button.waitForAnyPress();
		}
//		navigator.turnTo(0,true);
//		for(int i=0;i<4;i++){
//			navigator.travelTo(destination[i][0], destination[i][1]);
//		}
		robin.getLocalizer().doLocalization();
		Button.waitForAnyPress();

		robin.getOdometryCorrector().start();
		navigator.navigateMap(demo, true);
		Button.waitForAnyPress();
		//navigator.fireAt(9*segment, 9*segment, xMin, xMax, yMin, yMax, numBalls);
		
		
//		navigator.travelTo(60.96,60.96, true);
//		navigator.travelTo(0,0, true);
		navigator.turnToDeg(0,true);
		
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
