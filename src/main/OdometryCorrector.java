package main;
import lejos.nxt.Button;
import lejos.nxt.ColorSensor;
import lejos.nxt.Sound;


public class OdometryCorrector extends Thread { //TODO heading correction
	private static final long CORRECTION_PERIOD = 10;
	private static final double TILE_LENGTH = 30.48; // in centimeters\

	private static int lightThreshold; // Threshold between the light value of the line and that of the wooden floor	
	// x and y components of the distance between the midpoint between the wheels and each sensor (when the robot is facing 0 degrees)
	private static double xLSdistance;
	private static double xRSdistance;
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
	public static double leftErrorY;
	public static double avgErrorX=69;
	public static double avgErrorY=69;
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
	private boolean isActive;
	
	
	
	Object lock;
	Odometer odometer;
	LSController lsController;
	
	public OdometryCorrector(Odometer odometer, LSController lsController, Object lock) {
		this.lock = lock;
		this.odometer = odometer;
		this.lsController = lsController;
		// TODO: calibrate...
		xRSdistance = 3.45;
		xLSdistance = 2.8;
		yLSdistance = 13.6;
//		lightThreshold = 41;
//		lightThreshold = 45;
		lightThreshold = 50; // with shades
	}
	
	private void processLSData() { 
		
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
				leftSensorX = odometerX - yLSdistance*Math.sin(heading) - xLSdistance*Math.cos(heading);
				leftSensorY = odometerY - yLSdistance*Math.cos(heading) + xLSdistance*Math.sin(heading);
				
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
				rightSensorX = odometerX - yLSdistance*Math.sin(heading) + xRSdistance*Math.cos(heading);
				rightSensorY = odometerY - yLSdistance*Math.cos(heading) - xRSdistance*Math.sin(heading);
				
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

		 //if both sensors pass over an x gridline at the same time
			if(leftCrossingX&&rightCrossingX) {
				//Sound.beep();
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
				//Sound.twoBeeps();
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
				//Sound.beep();
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
		avgErrorX = errorX;
		//nav.stop();
		//Button.waitForAnyPress();
		odometer.setX(odometer.getX()- errorX);
		//nav.travelTo(-.35*TILE_LENGTH,5.6*TILE_LENGTH, false);
	}
	
	/**
	 * Corrects the x-position of the odometer (between the wheels) based on the sensor x-position
	 * It rounds the sensor position to the closest odd multiple of 15, and updates the odometer position based on 
	 * the distance between the sensor and the point between the wheels
	 * @param sensorX The x-position of the color sensor
	 */
	/*
	private void xRightCorrection(double sensorX) {
//		int roundSensorX = (int)Math.round(sensorX);
		double gridX;
		if(sensorX%TILE_LENGTH<TILE_LENGTH/2){
			gridX = ((int)(sensorX/TILE_LENGTH)) * TILE_LENGTH;
		}
		else{
			gridX = ((int)(sensorX/TILE_LENGTH) + 1) * TILE_LENGTH;
		}
		
		double correctX = gridX + yLSdistance*Math.sin(heading) - xLSdistance*Math.cos(heading);
		odometer.setX(correctX);
	}
	*/
	
	/**
	 * Corrects the x-position of the odometer (between the wheels) based on the sensor x-position
	 * It rounds the sensor position to the closest odd multiple of 15, and updates the odometer position based on 
	 * the distance between the sensor and the point between the wheels
	 * @param sensorX The x-position of the color sensor
	 */
	private void yCorrection(double errorY) {
		avgErrorY = errorY;
		//nav.stop();
		//Button.waitForAnyPress();
		odometer.setY(odometer.getY()- errorY);
		//nav.travelTo(-.35*TILE_LENGTH,5.6*TILE_LENGTH, false);
	}
	
	/**
	 * Corrects the y-position of the odometer (between the wheels) based on the sensor y-position
	 * It rounds the sensor position to the closest odd multiple of 15, and updates the odometer position based on 
	 * the distance between the sensor and the point between the wheels
	 * @param sensorY The y-position of the color sensor
	 */
	/*
	private void yRightCorrection(double sensorY) {
//		int roundSensorY = (int)Math.round(sensorY);
		double gridY;
		if(sensorY%TILE_LENGTH<TILE_LENGTH/2){
			gridY = ((int)(sensorY/TILE_LENGTH)) * TILE_LENGTH;
		}
		else{
			gridY = ((int)(sensorY/TILE_LENGTH) + 1) * TILE_LENGTH;
		}
		double correctY = gridY + yLSdistance*Math.cos(heading) + xLSdistance*Math.sin(heading);
		odometer.setY(correctY);
	}
	*/
	
	/**
	 * Corrects the y-position of the odometer (between the wheels) based on the sensor y-position
	 * It rounds the sensor position to the closest odd multiple of 15, and updates the odometer position based on 
	 * the distance between the sensor and the point between the wheels
	 * @param leftSensorY The y-position of the color sensor
	 */
	/*
	private void yAverageCorrection(double leftSensorY, double rightSensorY) {
		

	}
	
	/**
	 * Corrects the y-position of the odometer (between the wheels) based on the sensor y-position
	 * It rounds the sensor position to the closest odd multiple of 15, and updates the odometer position based on 
	 * the distance between the sensor and the point between the wheels
	 * @param leftSensorX The y-position of the color sensor
	 */
	/*
	private void xAverageCorrection(double leftSensorX, double rightSensorX) {
		
		
	}
	
	/**
	 * Heading correction when crossing a y grid line
	 * @param deltaY
	 */
	private void headingCorrection(double deltaX, double deltaY) {
		double correctHeading = Math.atan2(deltaX,deltaY);
		odometer.setHeading(correctHeading);
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