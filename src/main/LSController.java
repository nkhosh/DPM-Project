package main;
import lejos.nxt.ColorSensor;

/**
 * This class is controlling all the functionality of the light sensor,
 * for example, the reading of the sensor data is done through this class.
 * It protects the sensor objects and only gives read access to other classes.
 * @author Niloofar Khoshsiyar
 *
 */
public class LSController {
	private ColorSensor cs;

	public LSController(ColorSensor colorSensor) {
		cs = colorSensor;
	}
	
	/**
	 * Calibrates the light sensor using moving average method.
	 * @return true if the calibration is done successfully, false otherwise.
	 */
	public boolean calibrateColorSensor() {
		return true; 
	}
	
	
	/**
	 * Reads data from the light sensor.
	 * @return the light value returned by the sensor
	 */
	public int readCSdata(){
		return cs.getLightValue();
	}
	
	/**
	 * Activates the color sensor by turning on the flood light.
	 */
	public void activateCS(){
		cs.setFloodlight(true);
	}
	
	/**
	 * Switches off the flood light.
	 */
	public void deactivateCS(){
		cs.setFloodlight(false);
	}
	
	/**
	 * @return the normalized light value of the color sensor
	 */
	public int getNormalizedLightValue() {
		return cs.getNormalizedLightValue();
	}
}