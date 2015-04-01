package main;

import lejos.nxt.*;

public class Localizer {
	public static double ROTATION_SPEED = 30;
	public static final int LIMIT = 65;
	public static final double OFFSETY = 6.0;
	public static final double OFFSETX = 4.8;
	public static final int ERROR = 3;
	public static final int COMPENSATION = 65;
	public static int phase;
	public static double testAngle;
	public static double distance;
	public static int distanceFront;
	public static double angle;

	private Odometer odo;
	private Navigator nav;
	private UltrasonicSensor frontUSSensor,leftUSSensor;
	public static boolean isIncreasing;
	public static int distancePrevious;
	public static int distanceSide;
	public static int l;
	
	public static int r;
	
	public static double dupCounterL,dupCounterR;
	
	public ColorSensor left;
	public ColorSensor right;
	private final double sensorDL = Math.pow((Math.pow(12.65, 2)+Math.pow(3, 2)), 0.5);
	private final double sensorDR = Math.pow((Math.pow(12.65, 2)+Math.pow(3.55, 2)), 0.5);
	public static int gridCounter;
	public static int leftValue;
	public static int rightValue;
	public static double averageL;
	public static double averageR;
	public static int threshhold =40;
	public static int filterCountL;
	public static int filterCountR ;
	
	/**
	 * Constructor initializes the variables in the class
	 * @param odo Odometer object
	 * @param nav Navigator object
	 * @param us Array of two ultrasonic sensors
	 * @param ls Array of two color sensors
	 */
	public Localizer(Odometer odo, Navigator nav, UltrasonicSensor[] us, ColorSensor[] ls) {
		this.odo = odo;
		this.nav = nav;
		this.leftUSSensor = us[0];
		this.frontUSSensor = us[1];
		this.left = ls[0];
		this.right = ls[1];
		isIncreasing = true; 
		distancePrevious = -1;
	
		
		leftValue = -69;
		rightValue = -69;
		// switch off the ultrasonic sensor
		frontUSSensor.off();
		leftUSSensor.off();
	}
	
	/**
	 * 
	 */
	public void doLocalization() {
		
		doUSLocalization();
		doLSLocalization(0,0);
	}
	
	/**
	 * Localizes the robot using the ultrasonic sensors
	 */
	public void doUSLocalization(){		
		boolean isRunning = true;
		angle = 0;
		double xDistance = -1;
		double yDistance = -1;
		int delayCountLimit = 45;
		int delayCount = 0;
					
		while(isRunning)
		{
			int change = 0;
							
			nav.setRotationSpeed(ROTATION_SPEED);

			distanceFront = this.getFilteredData(frontUSSensor);
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
						 distanceSide = this.getFilteredData(leftUSSensor);
						
						 if(distanceSide < LIMIT)
						 {
							//Sound.beep();
							angle = odo.getHeadingDeg();
							//Button.waitForAnyPress();
							xDistance = this.getFilteredData(frontUSSensor) + OFFSETX;
							yDistance = distanceSide + OFFSETY;
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
			
	/**
	 * Localizes the robot using the color sensors given the approximate coordinates where it starts localization
	 * @param initialX
	 * @param initialY
	 */
	public void doLSLocalization(double initialX, double initialY){
		//assumes you are at point -7.5,-1 from your zero
		left.setFloodlight(true);
		right.setFloodlight(true);
		
		//travel to a relatively close point. I chose (-4,-4)
		//Sound.buzz();
		averageL = 0;
		filterCountL = 0;
		
		averageR = 0;
		filterCountR = 0;
		
		
		gridCounter = 0;
		  dupCounterL = 0;
		  dupCounterR = 0;
		  l = 0;
		  r = 0;
		 double gridAngleL[] = {0,0,0,0};
		 double gridAngleR[] = {0,0,0,0};
		 
		boolean leftDetected = false;
		boolean rightDetected = false;
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
			leftValue = left.getNormalizedLightValue();
			rightValue = right.getNormalizedLightValue();
			
			if(checkAgainstAvgL(leftValue)&&(dupCounterL > 50)) 
			{
				
				//Sound.beep();
				leftDetected = true;
				dupCounterL = 0;
				//nav.stop();
			//	Button.waitForAnyPress();
				//nav.setRotationSpeed(ROTATION_SPEED);
			}
			if(checkAgainstAvgR(rightValue)&&(dupCounterR > 50)) 

			{
				//Sound.buzz();
				rightDetected = true;
				dupCounterR = 0;
				//nav.stop();
				//Button.waitForAnyPress();
				//nav.setRotationSpeed(ROTATION_SPEED);
			}
			if(gridCounter == 8)
			{
				odo.getPosition(pos);
				if(Math.abs(startAngle - pos[2])<ERROR)
				{
					isRunning = false;
				}
			}		
			if (leftDetected || rightDetected)
			{
				
				
				if(leftDetected && l<4)
					{
					
						odo.getPosition(pos);
						gridAngleL[l] = pos[2];
						gridCounter ++;
						l++;
						leftDetected = false;
					}
				if(rightDetected && r<4)
				{
					
					odo.getPosition(pos);
					gridAngleR[r] = pos[2];
					gridCounter ++;
					r++;
					rightDetected = false;
					
				}
				
				

			}
			dupCounterL++;
			dupCounterR++;
			
			
			
		}
		//Sound.beep();
		nav.stop();
		odo.getPosition(pos);
		//this is the trig shown in the tutorial used to calculate the real X,Y and Theta
		double xTL =(gridAngleL[2] - gridAngleL[0]); 
		
		double yL = -sensorDL*Math.cos(Math.toRadians(xTL/2));
		
		double yTL = (gridAngleL[3] - gridAngleL[1]);
		
		double xL = -sensorDL*Math.cos(Math.toRadians(yTL/2));
		
		double deltaL = (yTL/2) + 90 - (gridAngleL[3]-180);
		
		
		
		double xTR =(gridAngleR[2] - gridAngleR[0]); 
		
		double yR = -sensorDR*Math.cos(Math.toRadians(xTR/2));
		
		double yTR = (gridAngleR[3] - gridAngleR[1]);
		
		double xR = -sensorDR*Math.cos(Math.toRadians(yTR/2));
		
		double deltaR = (yTR/2) + 90 - (gridAngleR[3]-180);
		
		
		
		double x = (xL + xR)/2;
		double y = (yL + yR)/2;
		double delta = (deltaL + deltaR)/2;
		

		//update these values, travel to (0,0), and turn to 0 degrees

		odo.setPosition(new double [] {x+ initialX -0.9, y+initialY + 1.5, pos[2]+delta }, new boolean [] {true, true, true});
		
		odo.getPosition(pos);
		nav.travelTo(initialX, initialY,false);
		odo.getPosition(pos);
		nav.turnTo(0,true);
		nav.stop();
	}
	
	private int getFilteredData(UltrasonicSensor s) {
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
	
	private boolean checkAgainstAvgL(int input)
	{
		if(averageL- input >threshhold)
			return true;
		else
		{
			double tmp = averageL * filterCountL;
			tmp+= input;
			filterCountL++;
			averageL = tmp/filterCountL;
			return false;
		}
		
		
	}
	private boolean checkAgainstAvgR(int input)
	{
		if(averageR - input >threshhold)
			return true;
		else
		{
			double tmp = averageR * filterCountR;
			tmp+= input;
			filterCountR++;
			averageR = tmp/filterCountR;
			return false;
		}
		
		
	}

}
