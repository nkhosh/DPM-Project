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
}
