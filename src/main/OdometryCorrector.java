package main;
import lejos.nxt.ColorSensor;


public class OdometryCorrector extends Thread { //TODO heading correction
	private static final long CORRECTION_PERIOD = 10;
	private static final int LIGHT_THRESHOLD = 140; // Threshold between the light value of the line and that of the wooden floor
	 // x and y components of the distance between the midpoint between the wheels and each sensor (when the robot is facing 0 degrees)
	private static double xLSdistance;
	private static double yLSdistance;

	double odometerX;
	double odometerY;
	double heading;
	
	Object lock;
	Odometer odometer;
	LSController lsController;
	
	public OdometryCorrector(Odometer odometer, LSController lsController, Object lock) {
		this.lock = lock;
		this.odometer = odometer;
		this.lsController = lsController;
		xLSdistance = 5;
		yLSdistance = 12.5;
	}
	
	private void processLSData() { 
		
	}
	
	public void run(){
		long correctionStart, correctionEnd;
		while (true) {
			correctionStart = System.currentTimeMillis();
			int leftSensorData = lsController.readFilteredLSdata()[0];
			int rightSensorData = lsController.readFilteredLSdata()[1];
			
			odometerX = odometer.getX();
			odometerY = odometer.getY();
			heading = odometer.getHeading();

			// if the sensor passes over a black line
			if( (leftSensorData < LIGHT_THRESHOLD && rightSensorData < LIGHT_THRESHOLD)) {
				
				// ... 
				
			}
			
			// if the sensor passes over a black line
			if( (leftSensorData < LIGHT_THRESHOLD)) {
				
				// Sensor position determined by trigonometry based on the rotation angle
				// theta and the distance from the sensor to the midpoint between the wheels
				double leftSensorX = odometerX - yLSdistance*Math.sin(heading) - xLSdistance*Math.cos(heading);
				double leftSensorY = odometerY - yLSdistance*Math.cos(heading) + xLSdistance*Math.sin(heading);
				
				double errorX = (leftSensorX%30)-15;
				double errorY = (leftSensorY%30)-15;
				
				// If the sensor is closer to a x grid line
				if(Math.abs(errorX) < Math.abs(errorY)){
					xLeftCorrection(leftSensorX);
				}
				// If the sensor is closer to a y grid line
				else{
					yLeftCorrection(leftSensorY);
				}
				
			}
			
			if( (rightSensorData < LIGHT_THRESHOLD)) {
				
				// Sensor position determined by trigonometry based on the rotation angle
				// theta and the distance from the sensor to the midpoint between the wheels
				double rightSensorX = odometerX - yLSdistance*Math.sin(heading) + xLSdistance*Math.cos(heading);
				double rightSensorY = odometerY + yLSdistance*Math.cos(heading) + xLSdistance*Math.sin(heading);
				
				double errorX = (rightSensorX%30)-15;
				double errorY = (rightSensorY%30)-15;
				
				// If the sensor is closer to a x grid line
				if(Math.abs(errorX) < Math.abs(errorY)){
					xRightCorrection(rightSensorX);
				}
				// If the sensor is closer to a y grid line
				else{
					yRightCorrection(rightSensorY);
				}
				
			}
			
			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD
							- (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometry correction will be
					// interrupted by another thread
				}
			}
		}
	}
	
	/**
	 * Corrects the x-position of the odometer (between the wheels) based on the sensor x-position
	 * It rounds the sensor position to the closest odd multiple of 15, and updates the odometer position based on 
	 * the distance between the sensor and the point between the wheels
	 * @param sensorX The x-position of the color sensor
	 */
	private void xLeftCorrection(double sensorX) {
		int roundSensorX = (int)Math.round(sensorX);
		double gridX;
		if(Math.abs((roundSensorX/15)%2)==1){
			gridX = (roundSensorX/15) * 15.0;
		}
		else{
			gridX = (roundSensorX/15 + 1) * 15.0;
		}
		
		double correctX = gridX + yLSdistance*Math.sin(heading) + xLSdistance*Math.cos(heading);
		odometer.setX(correctX);
	}
	
	/**
	 * Corrects the x-position of the odometer (between the wheels) based on the sensor x-position
	 * It rounds the sensor position to the closest odd multiple of 15, and updates the odometer position based on 
	 * the distance between the sensor and the point between the wheels
	 * @param sensorX The x-position of the color sensor
	 */
	private void xRightCorrection(double sensorX) {
		int roundSensorX = (int)Math.round(sensorX);
		double gridX;
		if(Math.abs((roundSensorX/15)%2)==1){
			gridX = (roundSensorX/15) * 15.0;
		}
		else{
			gridX = (roundSensorX/15 + 1) * 15.0;
		}
		
		double correctX = gridX + yLSdistance*Math.sin(heading) - xLSdistance*Math.cos(heading);
		odometer.setX(correctX);
	}
	
	/**
	 * Corrects the x-position of the odometer (between the wheels) based on the sensor x-position
	 * It rounds the sensor position to the closest odd multiple of 15, and updates the odometer position based on 
	 * the distance between the sensor and the point between the wheels
	 * @param sensorX The x-position of the color sensor
	 */
	private void yLeftCorrection(double sensorY) {
		int roundSensorY = (int)Math.round(sensorY);
		double gridY;
		if(Math.abs((roundSensorY/15)%2)==1){
			gridY = (roundSensorY/15) * 15.0;
		}
		else{
			gridY = (roundSensorY/15 + 1) * 15.0;
		}
		double correctY = gridY + yLSdistance*Math.cos(heading) - xLSdistance*Math.sin(heading);
		odometer.setY(correctY);
	}
	
	/**
	 * Corrects the y-position of the odometer (between the wheels) based on the sensor y-position
	 * It rounds the sensor position to the closest odd multiple of 15, and updates the odometer position based on 
	 * the distance between the sensor and the point between the wheels
	 * @param sensorY The y-position of the color sensor
	 */
	private void yRightCorrection(double sensorY) {
		int roundSensorY = (int)Math.round(sensorY);
		double gridY;
		if(Math.abs((roundSensorY/15)%2)==1){
			gridY = (roundSensorY/15) * 15.0;
		}
		else{
			gridY = (roundSensorY/15 + 1) * 15.0;
		}
		double correctY = gridY + yLSdistance*Math.cos(heading) + xLSdistance*Math.sin(heading);
		odometer.setY(correctY);
	}
}
