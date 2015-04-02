package main;

import lejos.nxt.UltrasonicSensor;

public class USMedianFilter extends Thread
{
	private UltrasonicSensor us;
	private int[] inputDistances, sortedDistances; //arrays to store the input distances from the sensor, and the sorted distances
	private int count, medianDistance;
	
	//The constructor
	public USMedianFilter(UltrasonicSensor us)
	{
		//setting up the initial variables
		this.us = us;
		inputDistances = new int[5];        //setting size of array to 5 
		sortedDistances = new int[5];		//setting size of array to 5	
		count = 0;							//setting initial count to 0
	}	
	
	//The filter polls the sensor 5 times
	public void runFilter()
	{
		for(int i=0; i < 5; i++)
		{
			inputDistances[count] = us.getDistance();	//putting the distance value into the input array
			count++;									//increment count
			count = count % 5; 							//this makes sure we don not exceed the array size
			sort(inputDistances);						//sort the input array 
			medianDistance = sortedDistances[2]; 		//gets the median value from the sorted array
		}
	}
	
	//using an insertion sort
	//this method takes the input array, sorts the current values in the array
	//and stores them in a sorted array, this way the original input array is never altered
	private void sort(int[] input)
	{		
		sortedDistances = input; 						//copying the inputs into the sorted array
		for(int i=1; i<sortedDistances.length; i++)
		{
			int temp = sortedDistances[i];
			int j;
			for (j=i-1; j>=0 && temp < sortedDistances[j]; j--)		
			{
				sortedDistances[j+1] = sortedDistances[j];			//swap the two values
			}
			sortedDistances[j+1] = temp;
		}
	}
	
	//getter method
	public int getFilteredDistance()
	{
		return this.medianDistance;
	}
}
