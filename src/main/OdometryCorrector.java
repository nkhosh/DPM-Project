package main;
import lejos.nxt.ColorSensor;


public class OdometryCorrector extends Thread {
	Object lock;
	Odometer odometer;
	LSController lsController;
	
	public OdometryCorrector(Odometer odometer, LSController lsController, Object lock) {
		this.lock = lock;
		this.odometer = odometer;
		this.lsController = lsController;
	}
	
	private void processLSData() { 
		
	}
}
