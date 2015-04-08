
package main;

import java.util.Arrays;

import lejos.nxt.UltrasonicSensor;

public class USController {
	UltrasonicSensor[] us;
	private static USMedianFilter[] usFilters;
	public static final int LOCALIZATION_FRONT_LIMIT = 70;
	
	public USController(UltrasonicSensor[] usSensor) {
		usFilters = new USMedianFilter[3];
		us = usSensor;
		for(int i=0; i<3; i++){
			usFilters[i] = new USMedianFilter( us[i] );
			usFilters[i].start();
		}
	}
	
	public int getDistance(int orientation) {
		us[orientation].ping();
		int distance = us[orientation].getDistance();
		try { Thread.sleep(15); } catch (InterruptedException e) {}
		return distance;
	}
	
	public int getFilteredDistance(int orientation){
		return usFilters[orientation].getFilteredDistance();
	}
	
	public void turnOff() {
		for(int i=0; i<us.length; i++) {
			us[i].off();
		}
	}
	
	
	public int getLocalizationFilteredData(int index) {
		int distance = 0;
		
		// do a ping
		us[index].ping();

		// wait for the ping to complete
		try { Thread.sleep(50); } catch (InterruptedException e) {}

		// there will be a delay here

		distance = us[index].getDistance();
		if (distance > LOCALIZATION_FRONT_LIMIT)
		{
			distance = LOCALIZATION_FRONT_LIMIT;
		}
		//I felt that anything above 30 was irrelevant, and 30 allowed for accuracy while letting me test with other bots in the area


		return distance;
	}
	
	
	/**
	 * TODO
	 *
	 */
	private class USMedianFilter extends Thread
	{
		private UltrasonicSensor us;
		private int medianDistance;
		private int windowSize;
		private int movingIndex;
		private int[] inputDistances;
		private int[] sortedDistances;
		
		
		public USMedianFilter(UltrasonicSensor us)
		{
			//setting up the initial variables
			this.us = us;
			windowSize = 3;
			inputDistances = new int[windowSize];
			sortedDistances = new int[windowSize];
			movingIndex = 0;
			
			for(int i=0; i<windowSize; i++){
				inputDistances[i] = 255;
				sortedDistances[i] = 255;
			}
		}	
		
		/**
		 *TODO
		 */
		public void run(){
			while(true){
				us.ping();
				int distance = us.getDistance();
				
				inputDistances[movingIndex] = distance;
				
				sortedDistances = Arrays.copyOf(inputDistances, windowSize);
				Arrays.sort(sortedDistances);

				movingIndex = (movingIndex+1) % windowSize;
				
				
				try { Thread.sleep(10); } catch (InterruptedException e) {}
			}
		}
		
		public int getFilteredDistance()
		{
			medianDistance = sortedDistances[windowSize/2];
			return medianDistance;
		}
	}

}