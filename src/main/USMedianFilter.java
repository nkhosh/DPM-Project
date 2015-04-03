package main;

import java.util.Arrays;

import lejos.nxt.UltrasonicSensor;

public class USMedianFilter extends Thread
{
	private UltrasonicSensor us;
	private int[] inputDistances, sortedDistances; //arrays to store the input distances from the sensor, and the sorted distances
	private int count, medianDistance;
	private int windowSize;
	private int movingIndex;
	private int[] distances;
	
	//The constructor
	public USMedianFilter(UltrasonicSensor us)
	{
		//setting up the initial variables
		this.us = us;
		windowSize = 5;
		distances = new int[windowSize];
		movingIndex = 0;
		
		for(int i=0; i<windowSize; i++){
			distances[i] = 255;
		}
	}	
	
//	//The filter polls the sensor 5 times
//	public void runFilter()
//	{
//		for(int i=0; i < windowSize; i++)
//		{
//			us.ping();
//			int distance = us.getDistance();
//			try { Thread.sleep(10); } catch (InterruptedException e) {}
//			
//			inputDistances[count] = distance;	//putting the distance value into the input array
//			count++;									//increment count
//			count = count % windowSize; 							//this makes sure we don not exceed the array size
//			sort(inputDistances);						//sort the input array 
//			medianDistance = sortedDistances[windowSize/2];  //gets the median value from the sorted array
//		}
//	}
	
	public void run(){
		while(true){
			us.ping();
			int distance = us.getDistance();
			
			distances[movingIndex] = distance;
			Arrays.sort(distances);

			movingIndex = (movingIndex+1) % windowSize;
			
			
			try { Thread.sleep(10); } catch (InterruptedException e) {}
		}
	}
	
	
	//using an insertion sort
	//this method takes the input array, sorts the current values in the array
	//and stores them in a sorted array, this way the original input array is never altered
//	private void sort(int[] input)
//	{		
//		sortedDistances = input; 						//copying the inputs into the sorted array
//		for(int i=1; i<sortedDistances.length; i++)
//		{
//			int temp = sortedDistances[i];
//			int j;
//			for (j=i-1; j>=0 && temp < sortedDistances[j]; j--)		
//			{
//				sortedDistances[j+1] = sortedDistances[j];			//swap the two values
//			}
//			sortedDistances[j+1] = temp;
//		}
//	}
	
	//getter method
	public int getFilteredDistance()
	{
		medianDistance = distances[windowSize/2];
		return medianDistance;
	}
	
//	public int runAndGetData(){
//		runFilter();
//		return this.medianDistance;
//	}
	
}
