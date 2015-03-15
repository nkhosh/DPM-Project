package main;
import lejos.nxt.ColorSensor;
import lejos.nxt.Sound;


public class OdometryCorrector extends Thread { //TODO heading correction
	private static final long CORRECTION_PERIOD = 10;
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
	private boolean leftCrossedX;
	private boolean leftCrossedY;
	private boolean rightCrossedX;
	private boolean rightCrossedY;
	
	Object lock;
	Odometer odometer;
	LSController lsController;
	
	public OdometryCorrector(Odometer odometer, LSController lsController, Object lock) {
		this.lock = lock;
		this.odometer = odometer;
		this.lsController = lsController;
		leftCrossedX = false;
		leftCrossedY = false;
		
		// TODO: calibrate...
		xLSdistance = 3;
		yLSdistance = 12.5;
		lightThreshold = 41;
	}
	
	private void processLSData() { 
		
	}
	
	public void run(){
		long correctionStart, correctionEnd;
		while (true) {
			correctionStart = System.currentTimeMillis();
			lsData = lsController.readFilteredLSdata();
			
			leftSensorData = lsData[0];
			rightSensorData = lsData[1];
			
			odometerX = odometer.getX();
			odometerY = odometer.getY();
			heading = odometer.getHeading();
			
			// if the sensor passes over a black line
			if(leftSensorData < lightThreshold) {
				Sound.beep();
				leftSensorX = odometerX - yLSdistance*Math.sin(heading) - xLSdistance*Math.cos(heading);
				leftSensorY = odometerY - yLSdistance*Math.cos(heading) + xLSdistance*Math.sin(heading);
				
				if (leftSensorX%30 > 15)
					leftErrorX = (leftSensorX%30) - 30;
				else
					leftErrorX = (leftSensorX%30);
				
				if (leftSensorY%30 > 15)
					leftErrorY = (leftSensorY%30) - 30;
				else
					leftErrorY = (leftSensorY%30);
				
				// If the sensor is closer to a x grid line
				if(Math.abs(leftErrorX) < Math.abs(leftErrorY)){
					xLeftCorrection(leftSensorX);
					
					if(!odometer.isTurning()){
						if(rightCrossedX){
							secondGridCrossingX = odometerX;
							secondGridCrossingY = odometerX;
//							xHeadingCorrection(secondGridCrossingX - firstGridCrossingX);
							rightCrossedX = false;
						}
						else{
							firstGridCrossingX = odometerX;
							firstGridCrossingY = odometerY;
							leftCrossedX = true;
						}
					}

					leftCrossedY = false;
					
				}
				// If the sensor is closer to a y grid line
				else{
					yLeftCorrection(leftSensorY);
					
					if(!odometer.isTurning()){
						if(rightCrossedY){
							secondGridCrossingX = odometerX;
							secondGridCrossingY = odometerX;
//							yHeadingCorrection(secondGridCrossingY - firstGridCrossingY);
							rightCrossedY = false;
						}
						else{
							firstGridCrossingX = odometerX;
							firstGridCrossingY = odometerY;
							leftCrossedY = true;
						}
					}

					leftCrossedX = false;
				}
				
			}
			
			if(rightSensorData < lightThreshold) {
				Sound.beep();
				rightSensorX = odometerX - yLSdistance*Math.sin(heading) + xLSdistance*Math.cos(heading);
				rightSensorY = odometerY - yLSdistance*Math.cos(heading) - xLSdistance*Math.sin(heading); /// + yLS..
				
				if (rightSensorX%30 > 15)
					rightErrorX = (rightSensorX%30) - 30;
				else
					rightErrorX = (rightSensorX%30);
				
				if (rightSensorY%30 > 15)
					rightErrorY = (rightSensorY%30) - 30;
				else
					rightErrorY = (rightSensorY%30);
				
				
				rightErrorX = (rightSensorX%30)-15;
				rightErrorY = (rightSensorY%30)-15;
				
				// If the sensor is closer to a x grid line
				if(Math.abs(rightErrorX) < Math.abs(rightErrorY)){
					xRightCorrection(rightSensorX);
					
					if(!odometer.isTurning()){
						if(leftCrossedX){
							secondGridCrossingX = odometerX;
							secondGridCrossingY = odometerX;
//							xHeadingCorrection(secondGridCrossingX - firstGridCrossingX);
							leftCrossedX = false;
						}
						else{
							firstGridCrossingX = odometerX;
							firstGridCrossingY = odometerY;
							rightCrossedX = true;
						}
					}

					rightCrossedY = false;
					
				}
				// If the sensor is closer to a y grid line
				else{
					yRightCorrection(rightSensorY);
					
					if(!odometer.isTurning()){
						if(leftCrossedY){
							secondGridCrossingX = odometerX;
							secondGridCrossingY = odometerX;
//							yHeadingCorrection(secondGridCrossingY - firstGridCrossingY);
							leftCrossedY = false;
						}
						else{
							firstGridCrossingX = odometerX;
							firstGridCrossingY = odometerY;
							rightCrossedY = true;
						}
					}

					rightCrossedX = false;
				}
				
			}


			// if both sensors pass over a grid line (x or y) at the same time
//			if( leftSensorData < lightThreshold && rightSensorData < lightThreshold && ( (leftCrossedX&&rightCrossedX)||(leftCrossedY&&rightCrossedY) ) ) {
//				// correct heading...
//				if(heading>=Math.PI*7/4 && heading<Math.PI/4)
//					heading = 0;
//				else if(heading>=Math.PI/4 && heading<Math.PI*3/4)
//					heading = Math.PI/2;
//				else if(heading>=Math.PI*3/4 && heading<Math.PI*5/4)
//					heading = Math.PI;
//				else
//					heading = Math.PI*3/2;
//				odometer.setHeading(heading);
//			}
//			
			
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
		if(roundSensorX%30<15){
			gridX = (roundSensorX/30) * 30.0;
		}
		else{
			gridX = (roundSensorX/30 + 1) * 30.0;
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
		if(roundSensorX%30<15){
			gridX = (roundSensorX/30) * 30.0;
		}
		else{
			gridX = (roundSensorX/30 + 1) * 30.0;
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
		if(roundSensorY%30<15){
			gridY = (roundSensorY/30) * 30.0;
		}
		else{
			gridY = (roundSensorY/30 + 1) * 30.0;
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
		if(roundSensorY%30<15){
			gridY = (roundSensorY/30) * 30.0;
		}
		else{
			gridY = (roundSensorY/30 + 1) * 30.0;
		}
		double correctY = gridY + yLSdistance*Math.cos(heading) + xLSdistance*Math.sin(heading);
		odometer.setY(correctY);
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
