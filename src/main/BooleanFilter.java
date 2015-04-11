package main;

import lejos.nxt.ColorSensor;

public class BooleanFilter implements Runnable {
	private ColorSensor cs;
	private int[] inputValues; //arrays to store the input light values from sensor
	private int count, mean;
	boolean valueDroped;
	private int THRESHOLD = 530;
	volatile boolean end = false;  
	
	
	//The constructor this sets up color sensor, input array and the count.
	public BooleanFilter(ColorSensor cs)
	{
		//setting up the initial variables
		this.cs = cs;
		inputValues = new int[5];           //setting size of array to 5
		count = 0;							//setting initial count to 0
	}
	
		
	
	public void run()
	{
		//while(!end)
		while(true)
		{
			inputValues[count] = cs.getNormalizedLightValue();	//putting the light value into the input array
			count++;											//increment count
			count = count % 5; 									//this makes sure we don not exceed the array size
			mean = getMean(inputValues);						//get the mean
			
			
			if (mean<THRESHOLD)									//if mean less than threshold return true
			{
				valueDroped = true;
			}
			else												//else return false
			{
				valueDroped = false;
			}
		}
	}
	
//	public void stopBoolFilter()
//	{
//		end = true;
//	}
	
	//keeps record of the current mean of the array
	private int getMean(int[] input)				
	{
		int total = 0; 
		int mean;
		for (int temp: input)
		{
			total = total + temp;
		}
		mean = total/input.length;
		return mean;				
	}	
	
	//getter method that returns true if value drop below  
	//the given normalized light value THRESHOLD 
	public boolean getBooleanFilterValue()						
	{
		return this.valueDroped;
	}
	
}


