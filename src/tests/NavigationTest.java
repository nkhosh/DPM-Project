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
		
		while(!touchSensor.isPressed());
		
		// Test travelTo
		navigator = robin.getNavigator();
		robin.getOdometer().start();
		robin.getOdometryCorrector().start();
		navigator.start();
		navigator.travelTo(0, 60);
		LCD.drawString("Navigator is running", 0, 0);
		
		// Test turnTo
		
		//Exit
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
		
	}
}
