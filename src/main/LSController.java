package main;
import lejos.nxt.ColorSensor;

public class LSController {
	ColorSensor cs[];
	public LSController(ColorSensor[] colorSensor) {
		cs = colorSensor;
	}
	/**
	 * Calibrates the light sensor using moving average method.
	 * @return true if the calibration is done successfully, false otherwise.
	 */
	public boolean calibrateLightSensor() {
		return true; //if the calibration is successful, false otherwise.
	}
	
	/**
	 * Reads filtered data from the light sensors
	 * @return an array of size 2 containing data from the left and right sensors respectively
	 */
	public int[] readFilteredLSdata(){ // TODO implement filter
		int[] data = new int[2];
		data[0] = cs[0].getLightValue();
		data[1] = cs[1].getLightValue();
		return data;
	}
}
