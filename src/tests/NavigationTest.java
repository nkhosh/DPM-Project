package tests;

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
		double[][] destination = {{0,60.96},{60.96,60.96},{0,0}};
		while(!touchSensor.isPressed());
		
		navigator = robin.getNavigator();
		robin.getOdometer().start();
		robin.getOdometryCorrector().start();
		robin.getLcdPrinter().start();
		navigator.setDestinationArray(destination);
		navigator.start();
	}
}
