
package main;
import lejos.nxt.UltrasonicSensor;


public class USController {
	private final static int LEFT=0, FRONT=1;
	private final static int DISTANCE_THRESHOLD = 20;
	UltrasonicSensor[] us;
	

	public USController(UltrasonicSensor[] usSensor) {
		us = usSensor;
	}
	
	public int getDistance(int orientation) {
		us[orientation].ping();
		int distance = us[orientation].getDistance();
		try { Thread.sleep(50); } catch (InterruptedException e) {}
		return distance;
	}


	
}