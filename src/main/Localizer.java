package main;

import lejos.nxt.*;

public class Localizer {
	//public enum LocalizationType { FALLING_EDGE, RISING_EDGE };
	public static double ROTATION_SPEED = 30;
	public static final int LIMIT = 70;
	public static final double OFFSETY = 6.0;
	public static final double OFFSETX = 4.8;
	public static final int ERROR = 3;
	public static final int COMPENSATION = 95;
	public static int phase;
	public static double testAngle;
	public static double distance;
	public static int distanceFront;
	public static double angle;

	private Odometer odo;
	private Navigator nav;
	private static UltrasonicSensor front,left;
	public static boolean isIncreasing;
	public static int distancePrevious;
	public static int distanceSide;
	
	public static double dupCounter;
	
	public ColorSensor ls;
	private final double sensorD = 13.6;//13
	public static int gridCounter;
	public static int lightValue;
	public static double average;
	public static int threshhold =30;
	public static int filterCount;
	
	
	public Localizer(Odometer odo, Navigator nav, UltrasonicSensor[] us, ColorSensor ls) {
		this.odo = odo;
		this.nav = nav;
		this.front = us[0];
		this.left = us[1];
		this.ls = ls;
		isIncreasing = true; 
		distancePrevious = -1;
	
		
		lightValue = -69;
		// switch off the ultrasonic sensor
		front.off();
		left.off();
	}
	
	public void doLocalization() {
		
		doUSLocalization();
		doLSLocalization(0,0);
	}
	
	public void doUSLocalization(){
		
		
			boolean isRunning = true;
			angle = 0;
			double xDistance = -1;
			double yDistance = -1;
			int delayCountLimit = 75;
			int delayCount = 0;
			
			
			while(isRunning)
			{
					int change = 0;
					
					
					
					nav.setRotationSpeed(ROTATION_SPEED);


					distanceFront = this.getFilteredData(front);
					if(distancePrevious == -1)
					{
						//Sound.beep();
					}
					else
					{
						change = distanceFront - distancePrevious;

							if (change > 0)
							{
								//Sound.beep();
								if(change >3 && delayCount > delayCountLimit)
								{
									delayCount = 0;
									Sound.beep();
									delayCount = 0;
									nav.stop();
									nav.turnTo(odo.getHeadingDeg()-COMPENSATION,true);
									//Button.waitForAnyPress();
									 distanceSide = this.getFilteredData(left);
									
									 if(distanceSide < LIMIT)
									 {
										//Sound.beep();
										angle = odo.getHeadingDeg();
										//Button.waitForAnyPress();
										
										yDistance = distanceSide + OFFSETY;
										
										nav.turnTo(odo.getHeadingDeg()+ 52 ,true);
										//Button.waitForAnyPress();
										xDistance = this.getFilteredData(front) + OFFSETX;
										isRunning = false;
									 }
									
								}
								
								isIncreasing = true;
							}
							else if (change < 0 )
							{
								isIncreasing = false;
							}
							delayCount ++;
							
					}
					distancePrevious = distanceFront;
					
			}		
					
		
			
			//double correctAngle = angle + COMPENSATION;
			nav.stop();
			//Sound.beep();
			double[] pos = {0,0,0};
			odo.getPosition(pos);
			odo.setPosition(new double [] {xDistance-30.48, yDistance-30.48, 270  }, new boolean [] {true, true, true});
			//nav.travelTo(-7.5, -1,false);
			nav.travelTo(-7.5, -1,false);
			nav.turnTo(0, true);
	}
			
	public void doLSLocalization(double xZero, double yZero){
		
		gridCounter = 0;
		double dupCounter = 0;
		int i = 0;
		double gridAngle[] = {0,0,0,0};
		average = 0;

		boolean isRunning = true;
		
		//first, start rotating. Then, each time you run through the loop, increment dupCounter. 
		// When you pass a black line and dupCounter is large enough, reset it and record that value in gridAngle. Also increment gridCounter.
		// dupCounter allows for a line to only be picked up once.
		// When gridCounter is 4, meaning you've passed 4 lines, AND the robot has made a full circle, then we exit the loop
		
		nav.setRotationSpeed(ROTATION_SPEED);
		double[] pos = new double[3];
		odo.getPosition(pos);		
		double startAngle = pos[2];
		
		
		while(isRunning)
		{
			lightValue = ls.getNormalizedLightValue();
			if(gridCounter == 4)
			{
				odo.getPosition(pos);
				if(Math.abs(startAngle - pos[2])<3)
				{
					isRunning = false;
				}
			}		
			 
			else if((checkAgainstAvg(lightValue))&&(dupCounter > 50))
			{
				odo.getPosition(pos);
				gridAngle[i] = pos[2];
				gridCounter ++;
				i++;
				dupCounter = 0;
			}
			lightValue = ls.getNormalizedLightValue(); 
			dupCounter++;
			
			
			
		}
		nav.stop();
		odo.getPosition(pos);
		//this is the trig shown in the tutorial used to calculate the real X,Y and Theta
		double xT =(gridAngle[2] - gridAngle[0]); 
		
		double y = -sensorD*Math.cos(Math.toRadians(xT/2));
		
		double yT = (gridAngle[3] - gridAngle[1]);
		
		double x = -sensorD*Math.cos(Math.toRadians(yT/2));
		
		double delta = (yT/2) + 90 - (gridAngle[3]-180);
		
		//update these values, travel to (0,0), and turn to 0 degrees
		odo.setPosition(new double [] {x + xZero + 0.2, y + yZero + 0.3, pos[2]+delta +1 }, new boolean [] {true, true, true});
		
		//Button.waitForAnyPress();
		odo.getPosition(pos);
		nav.travelTo(0, 0,false);
		//Button.waitForAnyPress();
		odo.getPosition(pos);
		nav.turnTo(0,true);

		
		nav.stop();
		
		
	}
	
	public int getFilteredData(UltrasonicSensor s) {
		 int distance = 0;
		
		// do a ping
		s.ping();

		// wait for the ping to complete
		try { Thread.sleep(50); } catch (InterruptedException e) {}
		
		// there will be a delay here
		
		distance = s.getDistance();
		if (distance > LIMIT)
		{
			distance = LIMIT;
		}
		//I felt that anything above 30 was irrelevant, and 30 allowed for accuracy while letting me test with other bots in the area

				
		return distance;
	}
	public boolean checkAgainstAvg(int input)
	{
		if((average -input) >threshhold)
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
