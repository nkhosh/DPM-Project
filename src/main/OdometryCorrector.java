package main;
import lejos.nxt.ColorSensor;
import lejos.nxt.Sound;


public class OdometryCorrector extends Thread { //TODO heading correction
	private static final long CORRECTION_PERIOD = 10;
	private static final double TILE_LENGTH = 30.48; // in centimeters

	private static int lightThreshold; // Threshold between the light value of the line and that of the wooden floor	
	// x and y components of the distance between the midpoint between the wheels and each sensor (when the robot is facing 0 degrees)
	private static double xLSdistance;
	private static double yLSdistance;

	// odometer position of the robot 
	private double odometerX;
	private double odometerY;
	private double heading;
	
	// light sensor data
	private int[] lsData;
	private int leftSensorData;
	private int rightSensorData;
	
	// position and position error of light sensors
	private double leftSensorX;
	private double leftSensorY;
	private double leftErrorX;
	private double leftErrorY;
	private double rightSensorX;
	private double rightSensorY;
	private double rightErrorX;
	private double rightErrorY;
	
	// data used to correct angle
	private double firstGridCrossingX;
	private double firstGridCrossingY;
	private double secondGridCrossingX;
	private double secondGridCrossingY;
	private double deltaX;
	private double deltaY;
	private boolean crossingGrid;
	private boolean leftCrossingX;
	private boolean leftCrossingY;
	private boolean rightCrossingX;
	private boolean rightCrossingY;
	
	Object lock;
	Odometer odometer;
	LSController lsController;
	
	public OdometryCorrector(Odometer odometer, LSController lsController, Object lock) {
		this.lock = lock;
		this.odometer = odometer;
		this.lsController = lsController;
		
		// TODO: calibrate...
//		xLSdistance = 3.15;
		xLSdistance = 3.75;
		yLSdistance = 13.5;
//		lightThreshold = 41;
//		lightThreshold = 45;
		lightThreshold = 50; // with shades
	}
	
	private void processLSData() { 
		
	}
	
	public void run(){
		lsController.activateLS();
		long correctionStart, correctionEnd;
		while (true) {
			correctionStart = System.currentTimeMillis();
			lsData = lsController.readFilteredLSdata();
			
			leftSensorData = lsData[0];
			rightSensorData = lsData[1];
			
			odometerX = odometer.getX();
			odometerY = odometer.getY();
			heading = odometer.getHeading();
			
			leftCrossingX = false;
			leftCrossingY = false;
			rightCrossingX = false;
			rightCrossingY = false;
			crossingGrid = false;
			
			// if the sensor passes over a black line
			if(leftSensorData < lightThreshold) {
				crossingGrid = true;
				leftSensorX = odometerX - yLSdistance*Math.sin(heading) - xLSdistance*Math.cos(heading);
				leftSensorY = odometerY - yLSdistance*Math.cos(heading) + xLSdistance*Math.sin(heading);
				
				if(leftSensorX%30.48 > 15.24)
					leftErrorX = (leftSensorX%30.48) - 30.48;
				else
					leftErrorX = (leftSensorX%30.48);
				
				if(leftSensorY%30.48 > 15.24)
					leftErrorY = (leftSensorY%30.48) - 30.48;
				else
					leftErrorY = (leftSensorY%30.48);
				
				// If the sensor is closer to a x grid line
				if(Math.abs(leftErrorX) < Math.abs(leftErrorY))
					leftCrossingX = true;
				// If the sensor is closer to a y grid line
				else
					leftCrossingY = true;
				
			}
			
			if(rightSensorData < lightThreshold) {
				crossingGrid = true;
				rightSensorX = odometerX - yLSdistance*Math.sin(heading) + xLSdistance*Math.cos(heading);
				rightSensorY = odometerY - yLSdistance*Math.cos(heading) - xLSdistance*Math.sin(heading);
				
				if(rightSensorX%30.48 > 15.24)
					rightErrorX = (rightSensorX%30.48) - 30.48;
				else
					rightErrorX = (rightSensorX%30.48);
				
				if(rightSensorY%30.48 > 15.24)
					rightErrorY = (rightSensorY%30.48) - 30.48;
				else
					rightErrorY = (rightSensorY%30.48);
				
				// If the sensor is closer to a x grid line
				if(Math.abs(rightErrorX) < Math.abs(rightErrorY))
					rightCrossingX = true;
				// If the sensor is closer to a y grid line
				else
					rightCrossingY = true;
				
			}


//			// if both sensors pass over an x gridline at the same time
//			if(leftCrossingX&&rightCrossingX) {
//				Sound.beep();
//				// correct heading (2 possibilities)
//				if(heading>=Math.PI/4 && heading<Math.PI*3/4)
//					heading = Math.PI/2;
//				else
//					heading = Math.PI*3/2;
//				odometer.setHeading(heading);
//				
//				// take an average correction of the position, using both sensors
//				xAverageCorrection(leftSensorX, rightSensorX);
//			}
//			
//			// if both sensors pass over a y gridline at the same time
//			else if(leftCrossingY&&rightCrossingY){
//				Sound.twoBeeps();
//				// correct heading (2 possibilities)
//				if(heading>=Math.PI*7/4 && heading<Math.PI/4)
//					heading = 0;
//				else if(heading>=Math.PI*3/4 && heading<Math.PI*5/4)
//					heading = Math.PI;
//				odometer.setHeading(heading);
//				
//				// take an average correction of the position, using both sensors
//				yAverageCorrection(leftSensorY, rightSensorY);
//			}
			
			
			// correct with both sensors separately
//			else if(crossingGrid){
			if(crossingGrid){
//				Sound.beep();
				if(leftCrossingX){
					xLeftCorrection(leftSensorX);
				}
				else if(leftCrossingY){
					yLeftCorrection(leftSensorY);
				}
				
				
				if(rightCrossingX)
					xRightCorrection(rightSensorX);
				else if(rightCrossingY)
					yRightCorrection(rightSensorY);		
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
	
	
//	private void positionCorrection(double leftSensorPosition, double rightSensorPosition){
//		int roundLeftSensorPosition = (int)Math.round(leftSensorPosition);
//		double leftGridPosition;
//		if(roundLeftSensorPosition%30.48<15){
//			leftGridPosition = (roundLeftSensorPosition/30) * 30.0;
//		}
//		else{
//			leftGridPosition = (roundLeftSensorPosition/30 + 1) * 30.0;
//		}
//		
//		int roundRightSensorPosition = (int)Math.round(rightSensorPosition);
//		double rightGridPosition;
//		if(roundRightSensorPosition%30<15){
//			rightGridPosition = (roundRightSensorPosition/30) * 30.0;
//		}
//		else{
//			rightGridPosition = (roundRightSensorPosition/30 + 1) * 30.0;
//		}
//		
//		
//		
//		if(leftCrossingX && rightCrossingX){
//		}
//		else if(leftCrossingY && rightCrossingY){
//			
//		}
//	}
	
	/**
	 * Corrects the x-position of the odometer (between the wheels) based on the sensor x-position
	 * It rounds the sensor position to the closest odd multiple of 15, and updates the odometer position based on 
	 * the distance between the sensor and the point between the wheels
	 * @param sensorX The x-position of the color sensor
	 */
	private void xLeftCorrection(double sensorX) {
//		int roundSensorX = (int)Math.round(sensorX);
		double gridX;
		if(sensorX%30.48<15.24){
			gridX = ((int)(sensorX/30.48)) * 30.48;
		}
		else{
			gridX = ((int)(sensorX/30.48) + 1) * 30.48;
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
//		int roundSensorX = (int)Math.round(sensorX);
		double gridX;
		if(sensorX%30.48<15.24){
			gridX = ((int)(sensorX/30.48)) * 30.48;
		}
		else{
			gridX = ((int)(sensorX/30.48) + 1) * 30.48;
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
//		int roundSensorY = (int)Math.round(sensorY);
		double gridY;
		if(sensorY%30.48<15.24){
			gridY = ((int)(sensorY/30.48)) * 30.48;
		}
		else{
			gridY = ((int)(sensorY/30.48) + 1) * 30.48;
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
//		int roundSensorY = (int)Math.round(sensorY);
		double gridY;
		if(sensorY%30.48<15.24){
			gridY = ((int)(sensorY/30.48)) * 30.48;
		}
		else{
			gridY = ((int)(sensorY/30.48) + 1) * 30.48;
		}
		double correctY = gridY + yLSdistance*Math.cos(heading) + xLSdistance*Math.sin(heading);
		odometer.setY(correctY);
	}
	
	/**
	 * Corrects the y-position of the odometer (between the wheels) based on the sensor y-position
	 * It rounds the sensor position to the closest odd multiple of 15, and updates the odometer position based on 
	 * the distance between the sensor and the point between the wheels
	 * @param leftSensorY The y-position of the color sensor
	 */
	private void yAverageCorrection(double leftSensorY, double rightSensorY) {
		
//		int roundLeftSensorY = (int)Math.round(leftSensorY);
		double leftGridY;
		if(leftSensorY%30.48 < 15.24){
			leftGridY = ((int)(leftSensorY/30.48)) * 30.48;
		}
		else{
			leftGridY = ((int)(leftSensorY/30.48) + 1) * 30.48;
		}
		double leftCorrectY = leftGridY + yLSdistance*Math.cos(heading) - xLSdistance*Math.sin(heading);
		
		
//		int roundRightSensorY = (int)Math.round(rightSensorY);
		double rightGridY;
		if(rightSensorY%30.48 < 15.24){
			rightGridY = ((int)(rightSensorY/30.48)) * 30.48;
		}
		else{
			rightGridY = ((int)(rightSensorY/30.48) + 1) * 30.48;
		}
		double rightCorrectY = rightGridY + yLSdistance*Math.cos(heading) + xLSdistance*Math.sin(heading);
		
		double averageCorrectY = (leftCorrectY + rightCorrectY)/2;
		
		odometer.setY(averageCorrectY);
	}
	
	/**
	 * Corrects the y-position of the odometer (between the wheels) based on the sensor y-position
	 * It rounds the sensor position to the closest odd multiple of 15, and updates the odometer position based on 
	 * the distance between the sensor and the point between the wheels
	 * @param leftSensorX The y-position of the color sensor
	 */
	private void xAverageCorrection(double leftSensorX, double rightSensorX) {
		
//		int roundLeftSensorX = (int)Math.round(leftSensorX);
		double leftGridX;
		if(leftSensorX%30.48 < 15.24){
			leftGridX = ((int)(leftSensorX/30.48)) * 30.48;
		}
		else{
			leftGridX = ((int)(leftSensorX/30.48) + 1) * 30.48;
		}
		double leftCorrectX = leftGridX + yLSdistance*Math.sin(heading) + xLSdistance*Math.cos(heading);
		
		
		int roundRightSensorX = (int)Math.round(rightSensorX);
		double rightGridX;
		if(roundRightSensorX%30.48<15.24){
			rightGridX = ((int)(roundRightSensorX/30.48)) * 30.48;
		}
		else{
			rightGridX = ((int)(roundRightSensorX/30.48) + 1) * 30.48;
		}
		double rightCorrectX = rightGridX + yLSdistance*Math.sin(heading) - xLSdistance*Math.cos(heading);
		
		double averageCorrectX = (leftCorrectX + rightCorrectX)/2;
		
		odometer.setX(averageCorrectX);
	}
	
	
	/**
	 * Heading correction when crossing a y grid line
	 * @param deltaY
	 */
	private void yHeadingCorrection(double deltaY) {
		double correctHeading;
		// heading in second or third quadrant
		if(heading>Math.PI/2 && heading<Math.PI*3/2)
			correctHeading = Math.PI - Math.asin((deltaY)/(2*xLSdistance));
		// heading in first or fourth quadrant
		else
			correctHeading = Math.asin((deltaY)/(2*xLSdistance));
		
		odometer.setHeading(correctHeading);
	}
	
	/**
	 * Heading correction when crossing an x grid line
	 * @param deltaY
	 */
	private void xHeadingCorrection(double deltaX) {
		double correctHeading;
		// heading in third or fourth quadrant
		if(heading>Math.PI && heading<Math.PI*2)
			correctHeading = -Math.acos((deltaX)/(2*xLSdistance));
		// heading in first or second quadrant
		else
			correctHeading = Math.acos((deltaX)/(2*xLSdistance));
		
		odometer.setHeading(correctHeading);
	}
}
