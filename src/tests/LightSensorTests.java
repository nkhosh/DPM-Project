package tests;

import lejos.nxt.LCD;
import main.LSController;
import main.Odometer;
import main.RobinHood;

public class LightSensorTests {
	Odometer odometer;
	private static LSController lsController;
	static int data;
	

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		RobinHood robinHood = new RobinHood();
		lsController = robinHood.getLScontroller();
		lsController.activateCS();
		
		while(true){
			data = lsController.readFilteredCSdata();
			
			LCD.drawString("LS:  "+ data, 0, 0);
		}

	}

}
