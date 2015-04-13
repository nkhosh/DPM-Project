package tests;

import lejos.nxt.LCD;
import main.LSController;
import main.Odometer;
import main.RobinHood;

/**
 * Test class to determine the light value threshold and to check the functionality of the light sensor.
 * @author Niloofar Khoshsiyar
 *
 */
public class LightSensorTests {
	Odometer odometer;
	private static LSController lsController;
	static int data;
	
	public static void main(String[] args) {
		RobinHood robinHood = new RobinHood();
		lsController = robinHood.getLScontroller();
		lsController.activateCS();
		
		while(true){
			data = lsController.readCSdata();
			
			LCD.drawString("LS:  "+ data, 0, 0);
		}

	}

}
