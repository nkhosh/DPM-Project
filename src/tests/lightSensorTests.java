package tests;

import lejos.nxt.LCD;
import main.LSController;
import main.Odometer;
import main.RobinHood;

public class lightSensorTests {
	Odometer odometer;
	private static LSController lsController;
	static int[] data;
	

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		RobinHood robinHood = new RobinHood();
		lsController = robinHood.getLScontroller();
		lsController.activateLS();
		
		while(true){
			data = lsController.readFilteredLSdata();
			
			LCD.drawString("LEFT:  "+ data[0], 0, 0);
			LCD.drawString("RIGHT: "+ data[1], 0, 1);
		}

	}

}
