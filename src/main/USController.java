
package main;
import lejos.nxt.UltrasonicSensor;


public class USController {
	UltrasonicSensor[] us;
	

	public USController(UltrasonicSensor[] usSensor) {
		us = usSensor;
	}
	
	public int getDistance(int orientation) {
		us[orientation].ping();
		int distance = us[orientation].getDistance();
		try { Thread.sleep(15); } catch (InterruptedException e) {}
		return distance;
	}
	
}