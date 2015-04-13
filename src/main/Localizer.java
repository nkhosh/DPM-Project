package main;

import lejos.nxt.*;

/**
 * This class is responsible for the localization of the robot.
 * It uses the ultrasonic or light sensors to localize the robot on the map.
 * @author Niloofar Khoshsiyar
 *
 */
public class Localizer {
	// Useful flags
	private final static int LEFT=0, FRONT=2;

	// Class constants
	private static final int FRONT_LIMIT = 60;
	private static final int SIDE_LIMIT = 45;
	private static final double OFFSETY = 6.0;
	private static final double OFFSETX = 4.8;
	private static final int LOW_ANGLE_OFFSET = 37;
	private static final int HIGH_ANGLE_OFFSET = 95;
	private static final int DISTANCE_CHANGE_THRESHOLD = 8;
	private static final int DELAY = 15;
	private static final int UP_COUNT_THRESHOLD = 50;
	private static final double SENSOR_DISTANCE = 13.6;
	private static final int MEAN_FILTER_THRESHOLD =30;
	
	// Class variables
	private static int distanceFront;
	private static int initialHeading = 270;
	private double rotationSpeed = 50;

	private int distancePrevious;
	private int distanceSide;

	private int gridCounter;
	private int lightValue;
	private double average;
	private int filterCount;
	private int derivation;

	// Class objects
	private USController usController;
	private LSController lsController;
	private Odometer odometer;
	private Navigator navigator;

	public Localizer(Odometer odo, Navigator navigator, USController usC, LSController lsC) {
		this.odometer = odo;
		this.navigator = navigator;
		this.usController = usC;
		this.lsController = lsC;

		distancePrevious = -1;
		lightValue = -69;
		
		// switch off the ultrasonic sensor
		usController.turnOff();
	}

	/**
	 * General localization method which performs ultrasonic and then light sensor localization in order
	 */
	public void doLocalization() {
		doUSLocalization();
		doLSLocalization(0,0);
	}

	/**
	 * Runs the ultrasonic localization.
	 */
	public void doUSLocalization(){
		rotationSpeed = 50;
		boolean isRunning = true;
		double xDistance = -1;
		double yDistance = -1;
		int delayCount = 0;
		distanceFront = 0;
		average = 0;
		filterCount = 0;
		derivation = 0;
		while(isRunning)
		{
			navigator.setHeadingRotationSpeed(rotationSpeed);

			if(distancePrevious == -1)
			{
				distancePrevious = usController.getLocalizationFilteredData(FRONT);
			}
			else
			{
				distanceFront = usController.getLocalizationFilteredData(FRONT);
				derivation = distanceFront - distancePrevious;

				if(derivation > DISTANCE_CHANGE_THRESHOLD && delayCount > DELAY)
				{
					navigator.stop();
					navigator.turnToDeg(odometer.getHeadingDeg()-LOW_ANGLE_OFFSET,true);
					distanceSide = usController.getLocalizationFilteredData(LEFT);

					if(distanceSide < SIDE_LIMIT)
					{
						yDistance = distanceSide + OFFSETY;

						navigator.turnToDeg(odometer.getHeadingDeg()+LOW_ANGLE_OFFSET ,true);
						xDistance = usController.getLocalizationFilteredData(FRONT) + OFFSETX;
						isRunning = false;
					}
					else
					{
						navigator.turnToDeg(odometer.getHeadingDeg()+HIGH_ANGLE_OFFSET,true);
						navigator.setHeadingRotationSpeed(rotationSpeed);
						delayCount = 0;
					}
				}

				distancePrevious = distanceFront;
				if (distancePrevious == FRONT_LIMIT)
					delayCount = 0;
			}
			delayCount++;
		}		

		double[] pos = {0,0,0};
		odometer.getPosition(pos);
		odometer.setPosition(new double [] {xDistance-navigator.getTileLength(), yDistance-navigator.getTileLength(), initialHeading  });
		navigator.travelTo(-5, -1,false);
		navigator.turnToDeg(30, true);
	}


	/**
	 * Runs the light localization.
	 * @param xZero Final destination x coordinate 
	 * @param yZero Final destination y coordinate
	 */
	private void doLSLocalization(double xZero, double yZero){
		lsController.activateCS();
		rotationSpeed = 30;
		gridCounter = 0;
		double distanceUpCounter = 0;
		int i = 0;
		double gridAngle[] = {0,0,0,0};
		boolean isRunning = true;

		//first, start rotating. Then, each time you run through the loop, increment dupCounter. 
		// When you pass a black line and dupCounter is large enough, reset it and record that value in gridAngle. Also increment gridCounter.
		// dupCounter allows for a line to only be picked up once.
		// When gridCounter is 4, meaning you've passed 4 lines, AND the robot has made a full circle, then we exit the loop
		
		navigator.setHeadingRotationSpeed(rotationSpeed);
		double[] pos = new double[3];
		odometer.getPosition(pos);		
		double startAngle = pos[2];


		while(isRunning)
		{
			lightValue = lsController.getNormalizedLightValue();
			if(gridCounter == 4)
			{
				odometer.getPosition(pos);
				if(Math.abs(startAngle - pos[2])<3)
				{
					isRunning = false;
				}
			}		

			else if((meanFilterCheck(lightValue))&&(distanceUpCounter > UP_COUNT_THRESHOLD))
			{
				Sound.beep();
				odometer.getPosition(pos);
				gridAngle[i] = pos[2];
				gridCounter ++;
				i++;
				distanceUpCounter = 0;
			}
			lightValue = lsController.getNormalizedLightValue(); 
			distanceUpCounter++;
		}

		odometer.getPosition(pos);
		
		//this is the trick shown in the tutorial used to calculate the real X,Y and Theta
		double xT =(gridAngle[2] - gridAngle[0]); 

		double y = -SENSOR_DISTANCE*Math.cos(Math.toRadians(xT/2));

		double yT = (gridAngle[3] - gridAngle[1]);

		double x = -SENSOR_DISTANCE*Math.cos(Math.toRadians(yT/2));

		double delta = (yT/2) + 90 - (gridAngle[3]-180);

		//update these values, travel to (0,0), and turn to 0 degrees
		odometer.setPosition(new double [] {x + xZero , y + yZero , pos[2]+delta  });

		odometer.getPosition(pos);
		navigator.travelTo(xZero, yZero,false);
		odometer.getPosition(pos);
		navigator.turnToDeg(0,true);
	}

	/**
	 * Compares the input with the average of the data
	 * @param input a reading value
	 * @return true if the input is below average, false otherwise
	 */
	private boolean meanFilterCheck(int input)
	{
		if((average -input) >MEAN_FILTER_THRESHOLD)
			return true;
		else
		{
			double tmp = average * filterCount;
			tmp+= input;
			filterCount++;
			average = tmp/filterCount;
			return false;
		}
	}
}

