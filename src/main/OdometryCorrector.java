package main;

public class OdometryCorrector extends Thread { //TODO heading correction
	private final static long CORRECTION_PERIOD = 10;
	private final static double TILE_LENGTH = 30.48; // in centimeters\

	// Threshold between the light value of the line and that of the wooden floor
	private final static int LIGHT_THRESHOLD = 50; 	
	
	// x and y components of the distance between the midpoint between the wheels and each sensor (when the robot is facing 0 degrees)
	private final static double X_LEFT_LS_DISTANCE = 3;
	private final static double X_RIGHT_LS_DISTANCE = 3.55;
	private final static double Y_LS_DISTANCE = 12.65;
	
	// odometer position of the robot 
	private double odometerX;
	private double odometerY;
	private double heading;
	
	// light sensor data
	private int[] lsData;
	private int leftSensorData;
	private int rightSensorData;
	
	// Position and position error of light sensors
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
	
	// Variables for heading corection with color sensor
	private boolean crossingGrid;
	private boolean leftCrossingX;
	private boolean leftCrossingY;
	private boolean rightCrossingX;
	private boolean rightCrossingY;
	private boolean isActive;
	private boolean rightAnglePath;
	
	// Variables determining movement while odometry correcting
	private boolean mustTurnLeft;
	private boolean mustTurnRight;

	Object lock;
	Odometer odometer;
	LSController lsController;
	
	public boolean getMustTurnLeft(){
		return mustTurnLeft;
	}
	
	public boolean getMustTurnRight(){
		return mustTurnRight;
	}
	
	public void setRightAnglePath(boolean bool){
		rightAnglePath = bool;
	}
	
	/**
	 * Constructor to build and initials the variables of odometry corrector
	 * @param odometer
	 * @param lsController
	 * @param lock
	 */
	public OdometryCorrector(Odometer odometer, LSController lsController, Object lock) {
		this.lock = lock;
		this.odometer = odometer;
		this.lsController = lsController;
		
//		xRightLSdistance = 3.45;
//		xLeftLSdistance = 2.8;
//		yLSdistance = 12.7;
		
		mustTurnLeft = false;
		mustTurnRight = false;
		
		rightAnglePath = false;
	}
	
	/**
	 * This methods runs the correction. It checks for the inputs from the color sensors
	 * and corrects the odometer values due to the calculations.
	 */
	public void run(){
		isActive = true;
		lsController.activateCS();
		long correctionStart, correctionEnd;
		
		while (true) {
			if(isActive){
				correctionStart = System.currentTimeMillis();
				lsData = lsController.readFilteredCSdata();
				
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
				
				if(rightAnglePath){
					if(leftSensorData < LIGHT_THRESHOLD && rightSensorData > LIGHT_THRESHOLD){
						mustTurnLeft = true;
					}
					else{
						mustTurnLeft = false;
					}
					
					if(rightSensorData < LIGHT_THRESHOLD && leftSensorData > LIGHT_THRESHOLD){
						mustTurnRight = true;
					}
					else{
						mustTurnRight = false;
					}
				}
				
				// if the sensor passes over a black line
				if(leftSensorData < LIGHT_THRESHOLD) {
					crossingGrid = true;
					leftSensorX = odometerX - Y_LS_DISTANCE*Math.sin(heading) - X_LEFT_LS_DISTANCE*Math.cos(heading);
					leftSensorY = odometerY - Y_LS_DISTANCE*Math.cos(heading) + X_LEFT_LS_DISTANCE*Math.sin(heading);
					correctionStart = System.currentTimeMillis();
					lsData = lsController.readFilteredCSdata();
					
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
					if(leftSensorData < LIGHT_THRESHOLD) {
						crossingGrid = true;
						leftSensorX = odometerX - Y_LS_DISTANCE*Math.sin(heading) - X_LEFT_LS_DISTANCE*Math.cos(heading);
						leftSensorY = odometerY - Y_LS_DISTANCE*Math.cos(heading) + X_LEFT_LS_DISTANCE*Math.sin(heading);
						
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
				
					if(rightSensorData < LIGHT_THRESHOLD) {
						crossingGrid = true;
						rightSensorX = odometerX - Y_LS_DISTANCE*Math.sin(heading) + X_RIGHT_LS_DISTANCE*Math.cos(heading);
						rightSensorY = odometerY - Y_LS_DISTANCE*Math.cos(heading) - X_RIGHT_LS_DISTANCE*Math.sin(heading);
						
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
	}
	
	/**
	 * Sets the activation status of the correction process
	 * @param activated
	 */
	public void setActive(boolean activated)
	{
		isActive = activated;
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
//	private void headingCorrection(double deltaX, double deltaY) {
//		double correctHeading = Math.atan2(deltaY,deltaX);
//		odometer.setHeading(correctHeading);
//	}
	
}