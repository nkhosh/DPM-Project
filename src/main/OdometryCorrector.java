package main;

public class OdometryCorrector extends Thread { //TODO heading correction
	private final static long CORRECTION_PERIOD = 10;
	private final static double TILE_LENGTH = 30.48; // in centimeters\

	// Threshold between the light value of the line and that of the wooden floor
	private final static int LIGHT_THRESHOLD = 50; 	
	
	// x and y components of the distance between the midpoint between the wheels and each sensor (when the robot is facing 0 degrees)
//	private final static double X_LEFT_LS_DISTANCE = 3;
//	private final static double X_RIGHT_LS_DISTANCE = 3.55;
//	private final static double LS_DISTANCE = 12.65;
	private final static double LS_DISTANCE = 11.8;
//	private final static double LS_DISTANCE = 12;
	
	// odometer position of the robot 
	private double odometerX;
	private double odometerY;
	private double heading;
	
	// light sensor data
	private int lsData;
//	private int sensorData;
	
	// Position and position error of light sensors
	private double sensorX;
	private double sensorY;
	private double errorX;
	private double errorY;
	
	private boolean isActive;
	
	// Variables determining movement while odometry correcting
//	private boolean mustTurnLeft;
//	private boolean mustTurnRight;

	Object lock;
	Odometer odometer;
	LSController lsController;
	
//	public boolean getMustTurnLeft(){
//		return mustTurnLeft;
//	}
//	
//	public boolean getMustTurnRight(){
//		return mustTurnRight;
//	}
//	
	
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
	}
	
	/**
	 * This methods runs the correction. It checks for the inputs from the color sensors
	 * and corrects the odometer values based on calculations.
	 */
	public void run(){
		isActive = true;
		lsController.activateCS();
		long correctionStart, correctionEnd;
		
		while (true) {
			if(isActive){
				correctionStart = System.currentTimeMillis();
				lsData = lsController.readFilteredCSdata();
				
				errorX = 0;
				errorY = 0;
				
				// if the sensor passes over a black line
				if(lsData < LIGHT_THRESHOLD) {
					odometerX = odometer.getX();
					odometerY = odometer.getY();
					heading = odometer.getHeading();
					
					sensorX = odometerX - LS_DISTANCE*Math.sin(heading);
					sensorY = odometerY - LS_DISTANCE*Math.cos(heading);
						
						if(sensorX%TILE_LENGTH > TILE_LENGTH/2)
							errorX = (sensorX%TILE_LENGTH) - TILE_LENGTH;
						else
							errorX = (sensorX%TILE_LENGTH);
						
						if(sensorY%TILE_LENGTH > TILE_LENGTH/2)
							errorY = (sensorY%TILE_LENGTH) - TILE_LENGTH;
						else
							errorY = (sensorY%TILE_LENGTH);
						
						// If the sensor is closer to a x grid line
						if(Math.abs(errorX) < Math.abs(errorY))
							xCorrection(errorX);
						// If the sensor is closer to a y grid line
						else
							yCorrection(errorY);
						
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
	
}