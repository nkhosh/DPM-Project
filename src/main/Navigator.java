package main;
import lejos.nxt.LCD;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.Sound;
import lejos.util.Delay;


public class Navigator{	
	// Variables for the speed of the movement
//	private final static int FAST_SPEED = 250, SLOW_SPEED = 120, NORMAL_SPEED=250;
	private final static int FAST_SPEED = 250, SLOW_SPEED = 120, NORMAL_SPEED=250, MIN_SPEED = 150, MAX_SPEED = 350;
	private final static int ROTATION_SPEED = 123;
	private static double wheelRadius; 
	private static double wheelsDistance;
	
	// The error and threshold values
	private final static double LOW_ANGLE_BANDWIDTH_DEG = 3;
	private final static double LOW_ANGLE_BANDWIDTH_RAD = Math.toRadians(LOW_ANGLE_BANDWIDTH_DEG);
	
	private final static double HIGH_ANGLE_BANDWIDTH_DEG = 5;
	private final static double HIGH_ANGLE_BANDWIDTH_RAD = Math.toRadians(HIGH_ANGLE_BANDWIDTH_DEG);
	// Used when the robot is possibly turning fast
	
	private final static double LOW_POSITION_BANDWIDTH = 1;
	private final static double HIGH_POSITION_BANDWIDTH = 30;
	

	// Some flags
	private final static int LEFT=0, RIGHT=1, FRONT=2;
	
	// Objects used by the navigator
	private Odometer odometer;
	private NXTRegulatedMotor[] wheels;
	private int[] speed;
	private USController usController;
	
	// Variables to indicate the position of RobinHood and the destination
	private Vector position, destination;
	private Vector unitOrientationVector;
	
	// Obstacle avoidance variables (in centimeters)
	private static final int FRONT_DISTANCE_THRESHOLD = 20, ANGLED_SENSOR_DISTANCE_THRESHOLD = 20, ANGLED_SENSOR_BANDCENTER = 23;
	private static final int FRONT_CORNER_DISTANCE_THRESHOLD = 25;
	private static final int ANGLED_SENSOR_DISTANCE_BANDWIDTH = 5;
	private static final int MAX_DISTANCE = 30;
	private static final int UPPER_ANGLED_BANDWIDTH = 5;
	private static final int LOWER_ANGLED_BANDWIDTH = 5;
	private static double rotationSpeed;
	
	private int[] distance = {255,255,255};
	private double launchAngle = Math.tan(7.4/(5*30.48 + 4));
	private double launchDistance = Math.sqrt(Math.pow(-7.4, 2) + Math.pow(5*30.48 + 4, 2));
	private Launcher launcher;
//	private int targetAngle;
	private double targetX;
	private double targetY;
	
	private int followingSide;
	private static final int P_SPEED_COEFFICIENT = 15;
	
	public Navigator(Odometer odometer, NXTRegulatedMotor[] wheels, USController usController, Launcher launch)	{
		this.launcher = launch;
		this.odometer = odometer;
		this.wheels = wheels;
		this.usController = usController;
		this.speed = new int[2];
		this.speed[LEFT] =  NORMAL_SPEED;
		this.speed[RIGHT] = NORMAL_SPEED;
		wheelRadius = odometer.getRadius();
		wheelsDistance = odometer.getWheelsDistance();
		destination = new Vector(0, 0);
		position = new Vector(0, 0);
		unitOrientationVector = new Vector();
		updatePosition();
		
		followingSide = LEFT;
		
		wheels[LEFT].setAcceleration(3500);
		wheels[RIGHT].setAcceleration(3500);
	}
	
	public int[] getDistance(){
		return distance;
	}
	
	public void setUSFollowingSide(int side){
		this.followingSide = side;
	}
	
	/**
	 * Moves the robot straight.
	 */
	protected void moveStraight() {

		wheels[LEFT].forward();
		wheels[RIGHT].forward();
		speed[LEFT]=NORMAL_SPEED;
		speed[RIGHT]=NORMAL_SPEED;
		
		updateSpeed();
	
	}
	
	/**
	 * Moves the robot to left.
	 */
	protected void moveLeft() {

		wheels[LEFT].forward();
		wheels[RIGHT].forward();
		
		speed[LEFT] = SLOW_SPEED;
		speed[RIGHT] = FAST_SPEED;
		updateSpeed();
		
	}
	
	/**
	 * Moves the robot to right.
	 */
	protected void moveRight() {

		wheels[LEFT].forward();
		wheels[RIGHT].forward();
		speed[LEFT] = FAST_SPEED;
		speed[RIGHT] = SLOW_SPEED;
		updateSpeed();
		
	}
	
	/**
	 * Turns right instantaneously when there is a close wall in front.
	 * Rotates about the center axis of the robot.
	 */
	protected void turnRight() {
		wheels[LEFT].forward();
		wheels[RIGHT].backward();
		speed[LEFT]=NORMAL_SPEED;
		speed[RIGHT]=NORMAL_SPEED;
		updateSpeed();
		

	}
	
	/**
	 * Turns left instantaneously when there is no wall in the left.
	 * Rotates about the center axis of the robot.
	 */
	protected void turnLeft() {
		wheels[LEFT].backward();
		wheels[RIGHT].forward();
		speed[RIGHT]=NORMAL_SPEED;
		speed[LEFT]=NORMAL_SPEED;
		updateSpeed();
		

	}
	
	protected void turn(int orientation){

		if(orientation == LEFT){
			wheels[LEFT].backward();
			wheels[RIGHT].forward();
		}
		else{
			wheels[LEFT].forward();
			wheels[RIGHT].backward();
		
		}
		
		speed[RIGHT]=NORMAL_SPEED;
		speed[LEFT]=NORMAL_SPEED;
		updateSpeed();
	}
	/**
	 * Move in the 
	 * @param orientation
	 */
	protected void move(int orientation){

		wheels[LEFT].forward();
		wheels[RIGHT].forward();
		
		if(orientation == LEFT){
			speed[LEFT]=SLOW_SPEED;
			speed[RIGHT]=FAST_SPEED;
		}
		else if(orientation == RIGHT){
			speed[LEFT]=FAST_SPEED;
			speed[RIGHT]=SLOW_SPEED;
		}
		else if(orientation == FRONT){
			speed[LEFT]=NORMAL_SPEED;
			speed[RIGHT]=NORMAL_SPEED;
		}
		
		
		updateSpeed();

	}
	
	protected void pMove(int orientation, int difference){

		wheels[LEFT].forward();
		wheels[RIGHT].forward();
		
		if(orientation == LEFT){
			speed[LEFT]=NORMAL_SPEED - pCalculateSpeed(difference);
			speed[RIGHT]=NORMAL_SPEED + pCalculateSpeed(difference);
		}
		else if(orientation == RIGHT){
			speed[LEFT]=NORMAL_SPEED + pCalculateSpeed(difference);
			speed[RIGHT]=NORMAL_SPEED - pCalculateSpeed(difference);
		}
		else if(orientation == FRONT){
			speed[LEFT]=NORMAL_SPEED;
			speed[RIGHT]=NORMAL_SPEED;
		}
		updateSpeed();
		

	}
	
	protected int pCalculateSpeed(int difference){
		return Math.abs(difference)*P_SPEED_COEFFICIENT;
	}
	
	
	protected void updateSpeed() {
		if(speed[LEFT] > MAX_SPEED){
			speed[LEFT] = MAX_SPEED;
 		}
		else if(speed[LEFT] < MIN_SPEED){
			speed[LEFT] = MIN_SPEED;
 		}
		
		if(speed[RIGHT] > MAX_SPEED){
			speed[RIGHT] = MAX_SPEED;
 		}
		else if(speed[RIGHT] < MIN_SPEED){
			speed[RIGHT] = MIN_SPEED;
 		}
		
		wheels[LEFT].setSpeed(speed[LEFT]);		     
		wheels[RIGHT].setSpeed(speed[RIGHT]);	
	}
	
	/**
	 * Moves the robot to the point that it has to launch the ball.
	 * It stops and turns to the target. Then calls the launcher to launch a ball.
	 * @param x
	 * @param y
	 * @param xMin
	 * @param xMax
	 * @param yMin
	 * @param yMax
	 * @param numBalls Number of ping pong balls to shoot
	 */
	public void fireAt(double x, double y, double xMin, double xMax, double yMin, double yMax,int numBalls)
	{
		int targetAngle;	//TODO: changed from a static variable to a method variable. Check for errors.
		targetAngle = -1;
		targetX = 0;
	    targetY = 0;
		for (int i = 0; i<90 ; i++)
		{
			double testX = -(Math.cos(Math.toRadians(i))*launchDistance);
			double testY = -(Math.sin(Math.toRadians(i))*launchDistance);
			
			testX = testX + x;
			testY = testY + y;
			if((testX > xMin && testX < xMax)&&(testY > yMin && testY < yMax))
			{
				targetX = testX;
				targetY = testY;
				targetAngle = 90-i;
				break;
			}
		}
		travelTo(targetX,targetY,false);
		turnToDeg(targetAngle + launchAngle,true);
		launcher.launch( numBalls);
	}
	
	/**
	 * Updates the current position by odometer values
	 */
	private void updatePosition() {
		position.setX(odometer.getX());
		position.setY(odometer.getY());
		unitOrientationVector.setOrientation(minimizeAngle(odometer.getHeading()));
	}
	
	/**
	 * Navigates the robot through the given map
	 * @param map The list of x and y coordinates of the destinations in the given map
	 */
	public void navigateMap(double[][]map, boolean obstacleAvoidance) {
		for(int i = 0; i < map.length ; i++)
			{
				travelTo(map[i][0],map[i][1], obstacleAvoidance);
			}
	}
	
	/**
	 * Navigates the robot through the given map
	 * @param map The list of x and y coordinates (as tile numbers)s of the destinations in the given map
	 */
	public void navigateMapTiles(double[][]map, boolean obstacleAvoidance) {
		for(int i = 0; i < map.length ; i++)
			{
				travelTo(map[i][0]*30.48,map[i][1]*30.48, obstacleAvoidance);
			}
	}
	
	/** 
	 * This function takes as arguments the x and y position in cm Will travel to designated position, while
	 * constantly updating it's heading
	 * @param x destination x coordinate
	 * @param y destination y coordinate
	 * @param obstacleAvoidance determines if the obstacle avoidance feature is activated or not
	 */
	public void travelTo(double x, double y,boolean obstacleAvoidance) {
		destination.setX(x);
		destination.setY(y);
		double relativeTargetOrientation = Math.atan2(x - odometer.getX(), y - odometer.getY()) ;
		long updateStart, updateEnd;
		
		/////////
		
//		double minAng;
//		while (Math.abs(x - odometer.getX()) > LOW_POSITION_BANDWIDTH || Math.abs(y - odometer.getY()) > LOW_POSITION_BANDWIDTH) {
//			minAng = (Math.atan2(x - odometer.getX(), y - odometer.getY())) * (180.0 / Math.PI);
//			if (minAng < 0)
//				minAng += 360.0;
//			this.turnTo(minAng, false);
//			this.moveStraight();
//		}
		
		/////////
		
		
		
		boolean possibleFrontObstacle = false;
		int frontObstacleDetectionCounter = 0, frontDistanceSum = 0;
		// If the robot isn't close enough to its destination
		while( (Math.abs(x - odometer.getX()) > LOW_POSITION_BANDWIDTH || Math.abs(y - odometer.getY()) > LOW_POSITION_BANDWIDTH) ) {
//			this.distance[FRONT] = usController.getDistance(FRONT);
//			this.distance[LEFT] = usController.getDistance(LEFT);
//			this.distance[RIGHT] = usController.getDistance(RIGHT);
			
			this.distance[FRONT] = usController.getFilteredDistance(FRONT);
			this.distance[LEFT] = usController.getFilteredDistance(LEFT);
			this.distance[RIGHT] = usController.getFilteredDistance(RIGHT);
			

			updateStart = System.currentTimeMillis();
			
			
			if(obstacleAvoidance && (Math.abs(x - odometer.getX()) > HIGH_POSITION_BANDWIDTH || Math.abs(y - odometer.getY()) > HIGH_POSITION_BANDWIDTH) ) {

//				if( possibleFrontObstacle ) {
//					frontObstacleDetectionCounter++;
//					frontDistanceSum += distance[FRONT];
//					
//					if( frontObstacleDetectionCounter>300 )
//						if( frontDistanceSum/300 > FRONT_CORNER_DISTANCE_THRESHOLD ) {
//							while(distance[FRONT] <= MAX_DISTANCE){
//								if(followingSide == LEFT){
//									turn(RIGHT);
//								}
//								else{
//									turn(LEFT);
//								}
//								distance[FRONT] = usController.getFilteredDistance(FRONT);
//							}
//							Delay.msDelay(800);
//							avoidObstacle(followingSide);
//							frontObstacleDetectionCounter=0;
//							possibleFrontObstacle=false;
//							frontDistanceSum = 0;
//						}
//						else {
//							possibleFrontObstacle = false;
//						}
//				}
				
				if( distance[FRONT] <= FRONT_DISTANCE_THRESHOLD ) {
					
					// Keep rotating until no obstacle in front
						while(distance[FRONT] <= MAX_DISTANCE){
							if(followingSide == LEFT){
								turn(RIGHT);
							}
							else{
								turn(LEFT);
							}
							distance[FRONT] = usController.getFilteredDistance(FRONT);
						}
						avoidObstacle(followingSide);
					}
				
				if( distance[FRONT] <= FRONT_CORNER_DISTANCE_THRESHOLD ) {
					Sound.beep();
					Delay.msDelay(700);
					distance[FRONT] = usController.getFilteredDistance(FRONT);
					
				// Keep rotating until no obstacle in front
					while(distance[FRONT] <= MAX_DISTANCE){
						if(followingSide == LEFT){
							turn(RIGHT);
						}
						else{
							turn(LEFT);
						}
						distance[FRONT] = usController.getFilteredDistance(FRONT);
					}
					Delay.msDelay(500);
					avoidObstacle(followingSide);
				}
//				else if( distance[FRONT] <= FRONT_CORNER_DISTANCE_THRESHOLD ) {
//					Delay.msDelay(600);
//					Sound.beep();
//					
//					// Keep rotating until no obstacle in front
//					while(distance[FRONT] <= MAX_DISTANCE){
//						if(followingSide == LEFT){
//							turn(RIGHT);
//						}
//						else{
//							turn(LEFT);
//						}
//						distance[FRONT] = usController.getFilteredDistance(FRONT);
//					}
//					avoidObstacle(followingSide);
//				}
				else if( distance[LEFT] <= ANGLED_SENSOR_DISTANCE_THRESHOLD ) {
					while(distance[LEFT] < ANGLED_SENSOR_BANDCENTER){
						turn(RIGHT);
						distance[LEFT] = usController.getFilteredDistance(LEFT);
					}
					followingSide = LEFT;
					avoidObstacle(LEFT);
				}
				else if( distance[RIGHT] <= ANGLED_SENSOR_DISTANCE_THRESHOLD ) {
					while(distance[RIGHT] < ANGLED_SENSOR_BANDCENTER){
						turn(LEFT);
						distance[RIGHT] = usController.getFilteredDistance(RIGHT);
					}
					followingSide = RIGHT;
					avoidObstacle(RIGHT);
				}
				else {
					turnToRad(relativeTargetOrientation, false);
					moveStraight();
				}
			}
			else
			{
				turnToRad(relativeTargetOrientation, false);
				moveStraight();
			}
			 
			relativeTargetOrientation = Math.atan2(x - odometer.getX(), y - odometer.getY());
			updateEnd = System.currentTimeMillis();
			
//			if (updateEnd - updateStart < 15) {
//				try {
//					Thread.sleep(15 - (updateEnd - updateStart));
//				} catch (InterruptedException e) {
//					// there is nothing to be done here because it is not
//					// expected that the odometer will be interrupted by
//					// another thread
//				}
//			}
		}
		
		// If the robot reached its destination
		wheels[LEFT].stop();
		wheels[RIGHT].stop();
	}
	
	
	/**
	 * TurnTo function which takes an angle and boolean as arguments The boolean controls whether or not to stop the
	 * motors when the turn is completed
	 * @param targetAngle The angle in degrees
	 */
	public void turnToDeg(double targetAngle, boolean stop) {
		turnToRad( Math.toRadians(targetAngle), stop);
//		double heading = odometer.getHeading();
//		
//		while(Math.abs(Odometer.minAngleFromTo(heading*180/Math.PI,targetAngle))>LOW_ANGLE_BANDWIDTH_DEG){
//
//			speed[LEFT] = ROTATION_SPEED;
//			speed[RIGHT] = ROTATION_SPEED;
//			
//			
//			// If the robot has to turn counterclockwise
//			if(Odometer.minAngleFromTo(heading*180/Math.PI,targetAngle)<0){
//				wheels[RIGHT].forward();
//				wheels[LEFT].backward();
//			}
//			// Else if the robot has to turn clockwise
//			else{
//				wheels[RIGHT].backward();
//				wheels[LEFT].forward();
//			}
//			
//
//			updateSpeed();
//
//			
//			heading = odometer.getHeading();
//			
//		}
//		if( stop )
//			this.stop();
		
	}
	
	/**
	 * TurnTo function which takes an angle and boolean as arguments The boolean controls whether or not to stop the
	 * motors when the turn is completed
	 * @param targetAngle The angle in degrees
	 */
	public void turnToRad(double targetAngle, boolean stop) {
		double heading = odometer.getHeading();
		
		while(Math.abs( Odometer.minimizeAngle(targetAngle-heading) ) > LOW_ANGLE_BANDWIDTH_RAD){

			speed[LEFT] = ROTATION_SPEED;
			speed[RIGHT] = ROTATION_SPEED;
			wheels[LEFT].setSpeed(speed[LEFT]);		     
			wheels[RIGHT].setSpeed(speed[RIGHT]);	
//			setRotationSpeed(30);
			
			// If the robot has to turn counterclockwise
			if( Odometer.minimizeAngle(targetAngle-heading) < 0){
				wheels[LEFT].backward();
				wheels[RIGHT].forward();
			}
			// Else if the robot has to turn clockwise
			else{
				wheels[LEFT].forward();
				wheels[RIGHT].backward();
			}
			

//			updateSpeed();

			
			heading = odometer.getHeading();
			
		}
		if( stop )
			this.stop();
		
	}
	
	public static double minAngleFromTo(double fromAngle, double toAngle) {
		double d = fixDegAngle(toAngle - fromAngle);
		
		if (d < 180.0)
			return d;
		else
			return d - 360.0;
	}
	
	public static double minAngleFromToRad(double fromAngle, double toAngle) {
		double d = fixRadAngle(toAngle - fromAngle);
//		double d = minimizeAngle(toAngle - fromAngle;
		if (d < Math.PI)
			return d;
		else
			return d - Math.PI*2;
	}
	
	public static double fixDegAngle(double angle) {		
		if (angle < 0.0)
			angle = 360.0 + (angle % 360.0);
		
		return angle % 360.0;
	}
	
	public static double fixRadAngle(double angle) {		
		if (angle < 0.0)
			angle = Math.PI*2 + (angle % (Math.PI*2));
		
		return angle % (Math.PI*2);
	}


	private void avoidObstacle(int followingSide){
		double relativeTargetOrientation = minimizeAngle( (destination.subtract(position)).getOrientation() );
		int distError;
		int oppositeSide;
		
		while(true){
			oppositeSide = (followingSide + 1) % 2;
			
			distance[followingSide] = usController.getFilteredDistance(followingSide);
			distance[oppositeSide] = usController.getFilteredDistance(oppositeSide);
			distance[FRONT] = usController.getFilteredDistance(FRONT);
			
//			this.distance[FRONT] = usController.getDistance(FRONT);
//			this.distance[followingSide] = usController.getDistance(followingSide);
//			this.distance[oppositeSide] = usController.getDistance(oppositeSide);
			
			
			updatePosition();
			distError = distance[followingSide]-ANGLED_SENSOR_BANDCENTER;
			
			relativeTargetOrientation = minimizeAngle( (destination.subtract(position)).getOrientation() );
			
			if( Math.abs(relativeTargetOrientation - (unitOrientationVector.getOrientation())) <= HIGH_ANGLE_BANDWIDTH_RAD
					 && distance[followingSide] > ANGLED_SENSOR_BANDCENTER ){
				break;
			}
			
			if( position.approxEquals(destination, HIGH_POSITION_BANDWIDTH) ) {
				break;
			}
			
			
			if(distance[FRONT] < FRONT_DISTANCE_THRESHOLD){
				break;
			}
			
			if(distance[oppositeSide] < ANGLED_SENSOR_DISTANCE_THRESHOLD){
				break;
			}
			
			
			/* Case 3: Positive error, moving too far from wall */
			if (distError > UPPER_ANGLED_BANDWIDTH) { // so the robot doesn't turn into the wall as much
//				move(followingSide);
				pMove(followingSide, distError);
			}
			
			
			/* Case 2: Negative error, moving too close to wall */
			else if (distError < - LOWER_ANGLED_BANDWIDTH) {
				if(followingSide == LEFT){
//					move(RIGHT);
					pMove(RIGHT, distError);
				}
				else if(followingSide == RIGHT){
//					move(LEFT);
					pMove(LEFT, distError);
				}
			}
			
			/* Case 1:  Error in bounds  */
			else /*if (Math.abs(distError) <= ANGLED_SENSOR_DISTANCE_BANDWIDTH)*/ { 
				move(FRONT);
			}

			
		}

		
	}


	
	/**
	 * Converts the traveling distance to the angle that the wheel has to rotate in degrees
	 * @param radius of the wheel
	 * @param distance that must be traveled
	 * @return
	 */
	public static double convertDistance(double radius, double distance) {
		return ((180.0 * distance) / (Math.PI * radius));
	}
	/**
	 * Converts angle that the robot should turn into the angle that the wheels have to turn in degrees
	 * @param radius of the wheel
	 * @param wheelDistance distance of the two wheels of the robot
	 * @param angle that we want the robot to change its heading to
	 * @return
	 */
	public static double convertAngle(double radius, double wheelDistance, double angle) {
		return convertDistance(radius, Math.PI * wheelDistance * angle / 360.0);
	}
	/**
	 * Minimizes an angle so that it is within the range from -PI to PI if not already in that range
	 * @param angle in radians
	 * @return
	 */
	private double minimizeAngle(double angle){
		double pi = Math.PI;
		angle = angle%(2*pi);
		double minimizedAngle = angle;
		if(angle<-pi){
			minimizedAngle = angle+2*pi;
		}
		else if(angle>pi){
			minimizedAngle = angle-2*pi;
		}
		return minimizedAngle;
	}
	
	/**
	 * Sets the speed of rotation of the robot
	 * @param speed
	 */
	public void setRotationSpeed(double speed) {
		rotationSpeed = speed;
		setSpeeds(rotationSpeed);
	}
	
	/**
	 * Sets the speed of each wheel according to the desired speed for the rotation of the robot
	 * @param rotationalSpeed
	 */
	public void setSpeeds(double rotationalSpeed) {
		double leftSpeed, rightSpeed;

		leftSpeed = (  rotationalSpeed * wheelsDistance * Math.PI / 360.0) *
				180.0 / (wheelRadius * Math.PI);
		rightSpeed = (-rotationalSpeed * wheelsDistance * Math.PI / 360.0) *
				180.0 / (wheelRadius * Math.PI);

		// set motor directions
		if (leftSpeed > 0.0)
			wheels[LEFT].forward();
		else {
			wheels[LEFT].backward();
			leftSpeed = -leftSpeed;
		}
		
		if (rightSpeed > 0.0)
			wheels[RIGHT].forward();
		else {
			wheels[RIGHT].backward();
			rightSpeed = -rightSpeed;
		}
		
		// set motor speeds
//		if (leftSpeed > 900.0)
//			wheels[LEFT].setSpeed(900);
//		else
//			wheels[LEFT].setSpeed((int)leftSpeed);
//		
//		if (rightSpeed > 900.0)
//			wheels[RIGHT].setSpeed(900);
//		else
//			wheels[RIGHT].setSpeed((int)rightSpeed);
		
		speed[LEFT] = (int)leftSpeed;
		speed[RIGHT] = (int)rightSpeed;
		
		updateSpeed();
	}
	
	
	/**
	 * Stops the robot
	 */
	public void stop(){
		wheels[RIGHT].stop();
		wheels[LEFT].stop();
		
//		wheels[RIGHT].setSpeed(0);
//		wheels[LEFT].setSpeed(0);
	}

	/**
	 * Implementation of a 2D vector.
	 *
	 */
	private class Vector {
		private double magnitude;
		private double orientation;
		private double x, y;
		public Vector(double x, double y) {
			setX(x);
			setY(y);
		}
		public Vector() {
			setMagnitude(1);
			setOrientation(0);
		}
		public Vector subtract(Vector v) {
			return new Vector(this.x-v.getX(), this.y-v.getY());
		}
		public double getX() {
			return x;
		}
		public double getY() {
			return y;
		}
		public double getOrientation() {
			return orientation;
		}
		public void setX(double x) {
			this.x = x;
			magnitude = Math.sqrt(x*x+y*y);
			orientation = Math.atan2(x, y);
		}
		public void setY(double y) {
			this.y = y;
			magnitude = Math.sqrt(x*x+y*y);
			orientation = Math.atan2(x, y);
		}
		public void setMagnitude(double magnitude) {
			this.magnitude = magnitude;
			this.x = magnitude*Math.sin(orientation);
			this.y = magnitude*Math.cos(orientation);
		}
		public void setOrientation(double orientation) {
			this.orientation = orientation;
			this.x = magnitude*Math.sin(orientation);
			this.y = magnitude*Math.cos(orientation);
		}
		/**
		 * Checks the approximate equality of the two vectors
		 * @param v The vector to be compared
		 * @return Returns true if the current Vector object's x and y coordinates
		 * are approximately equal to those of the parameter Vector v, false otherwise
		 */
		public boolean approxEquals(Vector v, double bandwidth) {
			return (Math.abs(this.x-v.getX())<= bandwidth &&
					Math.abs(this.y-v.getY())<=bandwidth);
		}
	}
	
}