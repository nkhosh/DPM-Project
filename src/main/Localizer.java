package main;

import lejos.nxt.*;
import lejos.util.Delay;

public class Localizer {
	//public enum LocalizationType { FALLING_EDGE, RISING_EDGE };
	private final static int LEFT=0, RIGHT=1, FRONT=2;
	
	public static double ROTATION_SPEED = 50;
	public static final int FRONTLIMIT = 60;
	public static final int SIDELIMIT = 45;
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
	public static int change;
	
	
	public Localizer(Odometer odo, Navigator nav, UltrasonicSensor[] us, ColorSensor ls) {
		this.odo = odo;
		this.nav = nav;
		this.front = us[FRONT];
		this.left = us[LEFT];
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
		
			ROTATION_SPEED = 50;
			boolean isRunning = true;
			angle = 0;
			double xDistance = -1;
			double yDistance = -1;
			//int delayCountLimit = 65;
			int delayCount = 0;
			distanceFront = 0;
			average = 0;
			filterCount = 0;
			
			change = 0;
			while(isRunning)
			{
					 
					
					
					
					nav.setHeadingRotationSpeed(ROTATION_SPEED);


					
					if(distancePrevious == -1)
					{
						distancePrevious = this.getFilteredData(front);
						//distanceFront = this.getFilteredData(front);
					}
					else
					{
						
						
						distanceFront = this.getFilteredData(front);
						
						change = distanceFront - distancePrevious;

								if(change > 8 && delayCount > 15)
								{
									
									//delayCount = 0;
									//Sound.beep();
									//delayCount = 0;
									nav.stop();
									//Button.waitForAnyPress();
									nav.turnToDeg(odo.getHeadingDeg()-COMPENSATION,true);
									//Button.waitForAnyPress();
									 distanceSide = this.getFilteredData(left);
									
									 if(distanceSide < SIDELIMIT)
									 {
										//Sound.beep();
										angle = odo.getHeadingDeg();
										//Button.waitForAnyPress();
										
										yDistance = distanceSide + OFFSETY;
										
										nav.turnToDeg(odo.getHeadingDeg()+ 37 ,true);
										//Button.waitForAnyPress();
										xDistance = this.getFilteredData(front) + OFFSETX;
										isRunning = false;
									 }
									 else
									 {
										 nav.turnToDeg(odo.getHeadingDeg()+COMPENSATION,true);
										 nav.setHeadingRotationSpeed(ROTATION_SPEED);
										 delayCount = 0;
									 }
									
								}

							distancePrevious = distanceFront;
							if (distancePrevious == FRONTLIMIT)
								delayCount = 0;
					}
					delayCount++;
					
					
			}		
					
		
			
			//double correctAngle = angle + COMPENSATION;
//			nav.stop();
			//Sound.beep();
			double[] pos = {0,0,0};
			odo.getPosition(pos);
			odo.setPosition(new double [] {xDistance-30.48, yDistance-30.48, 270  });
			nav.travelTo(-5, -1,false);
			//nav.travelTo(0, 0,false);
			nav.turnToDeg(30, true);
	}
	
			
	public void doLSLocalization(double xZero, double yZero){
		
		ls.setFloodlight(true);
		ROTATION_SPEED = 30;
		gridCounter = 0;
		double dupCounter = 0;
		int i = 0;
		double gridAngle[] = {0,0,0,0};
		
		
		
		boolean isRunning = true;
		
		//first, start rotating. Then, each time you run through the loop, increment dupCounter. 
		// When you pass a black line and dupCounter is large enough, reset it and record that value in gridAngle. Also increment gridCounter.
		// dupCounter allows for a line to only be picked up once.
		// When gridCounter is 4, meaning you've passed 4 lines, AND the robot has made a full circle, then we exit the loop
		
		nav.setHeadingRotationSpeed(ROTATION_SPEED);
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
				Sound.beep();
				odo.getPosition(pos);
				gridAngle[i] = pos[2];
				gridCounter ++;
				i++;
				dupCounter = 0;
			}
			lightValue = ls.getNormalizedLightValue(); 
			dupCounter++;
			
			
			
		}
//		nav.stop();
		odo.getPosition(pos);
		//this is the trig shown in the tutorial used to calculate the real X,Y and Theta
		double xT =(gridAngle[2] - gridAngle[0]); 
		
		double y = -sensorD*Math.cos(Math.toRadians(xT/2));
		
		double yT = (gridAngle[3] - gridAngle[1]);
		
		double x = -sensorD*Math.cos(Math.toRadians(yT/2));
		
		double delta = (yT/2) + 90 - (gridAngle[3]-180);
		
		//update these values, travel to (0,0), and turn to 0 degrees
		odo.setPosition(new double [] {x + xZero , y + yZero , pos[2]+delta  });
		
		//Button.waitForAnyPress();
		odo.getPosition(pos);
		nav.travelTo(xZero, yZero,false);
		//Button.waitForAnyPress();
		odo.getPosition(pos);
		nav.turnToDeg(0,true);

		
//		nav.stop();
		
		
	}
	
	public int getFilteredData(UltrasonicSensor s) {
		 int distance = 0;
		
		// do a ping
		s.ping();

		// wait for the ping to complete
		try { Thread.sleep(50); } catch (InterruptedException e) {}
		
		// there will be a delay here
		
		distance = s.getDistance();
		if (distance > FRONTLIMIT)
		{
			distance = FRONTLIMIT;
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

