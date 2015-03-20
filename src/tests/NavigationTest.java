package tests;

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
		navigator.travelTo(0, 60);
		
		// Test turnTo
		
		
	}
}
