package main;
import lejos.nxt.Button;
import lejos.nxt.ColorSensor;
import lejos.nxt.Sound;


public class OdometryCorrector extends Thread { //TODO heading correction
	private static final long CORRECTION_PERIOD = 10;
	private static final double TILE_LENGTH = 30.48; // in centimeters\

	private static int lightThreshold; // Threshold between the light value of the line and that of the wooden floor	
	// x and y components of the distance between the midpoint between the wheels and each sensor (when the robot is facing 0 degrees)
	private static double xLeftLSdistance;
	private static double xRightLSdistance;
	private static double yLSdistance;

	// odometer position of the robot 
	private double odometerX;
	private double odometerY;
	private double heading;
	private Navigator nav;
	
	// light sensor data
	private int[] lsData;
	private int leftSensorData;
	private int rightSensorData;
	
	// position and position error of light sensors
	private double leftSensorX;
	private double leftSensorY;
	private double leftErrorX;
	private double leftErrorY;
	public static double avgErrorX=69;
	public static double avgErrorY=69;
	private double rightSensorX;
	private double rightSensorY;
	private double rightErrorX;
	private double rightErrorY;
	
	
	private boolean crossingGrid;
	private boolean leftCrossingX;
	private boolean leftCrossingY;
	private boolean rightCrossingX;
	private boolean rightCrossingY;
	private boolean isActive;
	
	
	
	Object lock;
	Odometer odometer;
	LSController lsController;
	
	public OdometryCorrector(Odometer odometer, LSController lsController, Object lock) {
		this.lock = lock;
		this.odometer = odometer;
		this.lsController = lsController;
		
//		xRightLSdistance = 3.45;
//		xLeftLSdistance = 2.8;
//		yLSdistance = 12.7;
		
		xRightLSdistance = 3.55;
		xLeftLSdistance = 3;
		yLSdistance = 12.65;
		
		lightThreshold = 50;
	}
	
	
	public void run(){
		isActive = true;
		lsController.activateLS();
		long correctionStart, correctionEnd;
		
		while (true) {
			if(isActive){
			correctionStart = System.currentTimeMillis();
			lsData = lsController.readFilteredLSdata();
			
			leftSensorData = lsData[0];
			rightSensorData = lsData[1];
			
			leftErrorX = 0;
			leftErrorY = 0;
			rightErrorX = 0;
			rightErrorY = 0;
			
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
				leftSensorX = odometerX - yLSdistance*Math.sin(heading) - xLeftLSdistance*Math.cos(heading);
				leftSensorY = odometerY - yLSdistance*Math.cos(heading) + xLeftLSdistance*Math.sin(heading);
				
				if(leftSensorX%TILE_LENGTH > TILE_LENGTH/2)
					leftErrorX = (leftSensorX%TILE_LENGTH) - TILE_LENGTH;
				else
					leftErrorX = (leftSensorX%TILE_LENGTH);
				
				if(leftSensorY%TILE_LENGTH > TILE_LENGTH/2)
					leftErrorY = (leftSensorY%TILE_LENGTH) - TILE_LENGTH;
				else
					leftErrorY = (leftSensorY%TILE_LENGTH);
				
				// If the sensor is closer to a x grid line
				if(Math.abs(leftErrorX) < Math.abs(leftErrorY))
					leftCrossingX = true;
				// If the sensor is closer to a y grid line
				else
					leftCrossingY = true;
				
			}
			
			if(rightSensorData < lightThreshold) {
				crossingGrid = true;
				rightSensorX = odometerX - yLSdistance*Math.sin(heading) + xRightLSdistance*Math.cos(heading);
				rightSensorY = odometerY - yLSdistance*Math.cos(heading) - xRightLSdistance*Math.sin(heading);
				
				if(rightSensorX%TILE_LENGTH > TILE_LENGTH/2)
					rightErrorX = (rightSensorX%TILE_LENGTH) - TILE_LENGTH;
				else
					rightErrorX = (rightSensorX%TILE_LENGTH);
				
				if(rightSensorY%TILE_LENGTH > TILE_LENGTH/2)
					rightErrorY = (rightSensorY%TILE_LENGTH) - TILE_LENGTH;
				else
					rightErrorY = (rightSensorY%TILE_LENGTH);
				
				// If the sensor is closer to a x grid line
				if(Math.abs(rightErrorX) < Math.abs(rightErrorY))
					rightCrossingX = true;
				// If the sensor is closer to a y grid line
				else
					rightCrossingY = true;
				
			}

		 // if both sensors pass over an x gridline at the same time
			if(leftCrossingX&&rightCrossingX) {
				// correct heading (2 possibilities)
				if(heading>=Math.toRadians(0) && heading<Math.toRadians(180))
					heading = Math.toRadians(90);
				else
					heading = Math.toRadians(270);
				odometer.setHeading(heading);
				
				// take an average correction of the position, using both sensors
				xCorrection((leftErrorX + rightErrorX)/2);
			}
			
			// if both sensors pass over a y gridline at the same time
			else if(leftCrossingY&&rightCrossingY){
				// correct heading (2 possibilities)
				if(heading>=Math.toRadians(90) && heading<Math.toRadians(270))
					heading = Math.toRadians(180);
				else
					heading = 0;
				odometer.setHeading(heading);
				
				// take an average correction of the position, using both sensors
				yCorrection((leftErrorY + rightErrorY)/2);
			}			
			
			// correct with both sensors separately
			else if(crossingGrid){
				
				if(leftCrossingX){
					xCorrection(leftErrorX);
				}
				else if(leftCrossingY){
					yCorrection(leftErrorY);
				}
				
				
				if(rightCrossingX)
					xCorrection(rightErrorX);
				else if(rightCrossingY)
					yCorrection(rightErrorY);	
				
				// heading correction if crossing x and y grid lines at the same time
//				if(leftCrossingX && rightCrossingY){
//					headingCorrection(rightErrorX,leftErrorY);
//				}
//				
//				else if(rightCrossingX && leftCrossingY){
//					headingCorrection(-leftErrorX,-rightErrorY);
//				}
				
				
				
				
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
	}
	public void setActive(boolean bool)
	{
		isActive = bool;
	}
	
	/**
	 * Corrects the x-position of the odometer (between the wheels) based on the sensor x-position
	 * It rounds the sensor position to the closest odd multiple of 15, and updates the odometer position based on 
	 * the distance between the sensor and the point between the wheels
	 * @param sensorX The x-position of the color sensor
	 */
	private void xCorrection(double errorX) {
		odometer.setX(odometerX - errorX);
	}
	
	/**
	 * Corrects the x-position of the odometer (between the wheels) based on the sensor x-position
	 * It rounds the sensor position to the closest odd multiple of 15, and updates the odometer position based on 
	 * the distance between the sensor and the point between the wheels
	 * @param sensorX The x-position of the color sensor
	 */
	private void yCorrection(double errorY) {
		odometer.setY(odometerY - errorY);
	}
	/**
	 * Heading correction when each sensor crosses an opposite gridline
	 * @param deltaY
	 */
	private void headingCorrection(double deltaX, double deltaY) {
		double correctHeading = Math.atan2(deltaY,deltaX);
		odometer.setHeading(correctHeading);
	}
	
}