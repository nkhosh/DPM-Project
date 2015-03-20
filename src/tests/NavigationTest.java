package tests;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.SensorPort;
import lejos.nxt.TouchSensor;
import main.Navigator;
import main.RobinHood;

public class NavigationTest {
	public static void main(String[] args) {
		RobinHood robin = new RobinHood();
		Navigator navigator;
		SensorPort port = SensorPort.S4;
		TouchSensor touchSensor = new TouchSensor(port);
//		double[][] destination = {{60,60}};
		while(!touchSensor.isPressed());
		
		// Test travelTo
		navigator = robin.getNavigator();
		robin.getOdometer().start();
		robin.getOdometryCorrector().start();
		robin.getLcdPrinter().start();
//		navigator.setDestinationArray(destination);
		navigator.start();
		navigator.turnTo(180);
//		navigator.travelTo(0, 60);
		
		// Test turnTo
		
		//Exit
		while(true) { if(touchSensor.isPressed()) break;}
		System.exit(0);
	}
}
