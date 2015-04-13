package main;

import java.util.Arrays;
import lejos.nxt.UltrasonicSensor;

/**
 * This class is controlling all the functionality of the ultrasonic sensors,
 * for example, the reading of the sensor data is done through this class.
 * It protects the sensor objects and only gives read access to other classes.
 * Also, the median filtering of the ultrasonic data is done through a private class inside this object.
 * @author Niloofar Khoshsiyar
 *
 */
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
	
	/**
	 * Returns the distance read by the sensor in the given orientation
	 * @param orientation of the sensor
	 * @return the distance that the sensor reads
	 */
	public int getDistance(int orientation) {
		us[orientation].ping();
		int distance = us[orientation].getDistance();
		try { Thread.sleep(15); } catch (InterruptedException e) {}
		return distance;
	}
	
	/**
	 * Returns the filtered distance of the sensor in the given orientation
	 * @param orientation of the sensor
	 * @return the distance of the sensor after median filter has been applied
	 */
	public int getFilteredDistance(int orientation){
		return usFilters[orientation].getFilteredDistance();
	}
	
	/**
	 * Turns off the ultrasonic sensors
	 */
	public void turnOff() {
		for(int i=0; i<us.length; i++) {
			us[i].off();
		}
	}
	
	/**
	 * Used by localization to filter the data appropriately
	 * @param orientation of the sensor
	 * @return the distance after the specific filter has been applied
	 */
	protected int getLocalizationFilteredData(int orientation) {
		int distance = 0;
		
		// do a ping
		us[orientation].ping();

		// wait for the ping to complete
		try { Thread.sleep(50); } catch (InterruptedException e) {}

		// there will be a delay here

		distance = us[orientation].getDistance();
		if (distance > LOCALIZATION_FRONT_LIMIT)
		{
			distance = LOCALIZATION_FRONT_LIMIT;
		}
		//I felt that anything above 30 was irrelevant, and 30 allowed for accuracy while letting me test with other bots in the area


		return distance;
	}
	
	
	/**
	 * This class implements functions used for running median filter on the sensor data.
	 * It gets data from the sensors and sorts them in an array.
	 * Then change the middle value by the median of the elements.
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
		
		//The constructor
		public USMedianFilter(UltrasonicSensor us)
		{
			//setting up the initial variables
			this.us = us;
			windowSize = 1;
			inputDistances = new int[windowSize];
			sortedDistances = new int[windowSize];
			movingIndex = 0;
			
			for(int i=0; i<windowSize; i++){
				inputDistances[i] = 255;
				sortedDistances[i] = 255;
			}
		}	
		
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