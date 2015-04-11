package main;
import lejos.nxt.ColorSensor;

public class LSController {
	ColorSensor cs;
	public static double average;
	public static int threshhold =30;
	public static int filterCount;
	
	
	public LSController(ColorSensor colorSensor) {
		cs = colorSensor;
	}
	/**
	 * Calibrates the light sensor using moving average method.
	 * @return true if the calibration is done successfully, false otherwise.
	 */
	public boolean calibrateColorSensor() {
		return true; //if the calibration is successful, false otherwise.
	}
	
	
	/**
	 * Reads filtered data from the light sensors
	 * @return an array of size 2 containing data from the left and right sensors respectively
	 */
	public int readFilteredCSdata(){ // TODO implement filter
		int data;
		data = cs.getLightValue();
		return data;
	}
	
	public void activateCS(){
		average = 0;
		cs.setFloodlight(true);
	}
	
	public void deactivateCS(){
		cs.setFloodlight(false);
	}
	
	public boolean checkAgainstAvg(int input)
	{
		if((average -input) >threshhold)
			return true;
		else
		{
			double tmp = average * filterCount;
			tmp+= input;
			filterCount++;
			average = tmp/filterCount;
			return false;
		}
		
		
	}
}