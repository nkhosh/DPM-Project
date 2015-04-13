package main;

/**
 * This class runs along with the Odometer to correct the reported values in case of minor errors.
 * It scans the gridlines on the map by a light sensor to correct the position information of odometer.
 * @author Niloofar Khoshsiyar
 *
 */
public class OdometryCorrector extends Thread {

	// Threshold between the light value of the line and that of the wooden floor
	private final static int LIGHT_THRESHOLD = 50;
	
	private final static long CORRECTION_PERIOD = 10; //ms
	private final static double LS_TO_CENTER_DISTANCE = 11.8; //cm
	private final static double TILE_LENGTH = 30.48; //cm
	
	// odometer position variables of the robot 
	private double odometerX;
	private double odometerY;
	private double heading;
	
	// Class variables
	private boolean isActive;
	private Odometer odometer;
	
	// Light sensor variables
	private LSController lsController;	
	private int lsData;
	
	// Position and position error of light sensor
	private double sensorX;
	private double sensorY;
	private double errorX;
	private double errorY;
	
	/**
	 * Constructor to build and initials the variables of odometry corrector
	 * @param odometer
	 * @param lsController
	 * @param lock
	 */
	public OdometryCorrector(Odometer odometer, LSController lsController, Object lock) {
		this.odometer = odometer;
		this.lsController = lsController;
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
				lsData = lsController.readCSdata();
				
				errorX = 0;
				errorY = 0;
				
				// if the sensor passes over a black line
				if(lsData < LIGHT_THRESHOLD) {
					odometerX = odometer.getX();
					odometerY = odometer.getY();
					heading = odometer.getHeading();
					
					sensorX = odometerX - LS_TO_CENTER_DISTANCE*Math.sin(heading);
					sensorY = odometerY - LS_TO_CENTER_DISTANCE*Math.cos(heading);
						
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
	 * @param errorX The x-position of the color sensor
	 */
	private void xCorrection(double errorX) {
		odometer.setX(odometerX - errorX);
	}
	
	/**
	 * Corrects the y-position of the odometer (between the wheels) based on the sensor y-position
	 * It rounds the sensor position to the closest odd multiple of 15, and updates the odometer position based on 
	 * the distance between the sensor and the point between the wheels
	 * @param errorY The y-position of the color sensor
	 */
	private void yCorrection(double errorY) {
		odometer.setY(odometerY - errorY);
	}
	
}