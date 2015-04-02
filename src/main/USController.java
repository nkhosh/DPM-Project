
package main;
import lejos.nxt.UltrasonicSensor;


public class USController {
	UltrasonicSensor[] us;
	private static USMedianFilter[] usFilters;
	private final static int LEFT=0, RIGHT=1, FRONT=2;
	

	public USController(UltrasonicSensor[] usSensor) {
		usFilters = new USMedianFilter[3];
		us = usSensor;
		for(int i=0; i<3; i++){
			usFilters[i] = new USMedianFilter( us[i] );
		}
	}
	
	public int getDistance(int orientation) {
		us[orientation].ping();
		int distance = us[orientation].getDistance();
		try { Thread.sleep(15); } catch (InterruptedException e) {}
		return distance;
	}
	
	public int getFilteredDistance(int orientation){
		usFilters[orientation].runFilter();
		return usFilters[orientation].getFilteredDistance();
	}
	
}