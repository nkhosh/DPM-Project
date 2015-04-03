package main;
import lejos.nxt.LCD;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.Sound;


public class Navigator{	
	// Variables for the speed of the movement
	private final static int FAST_SPEED = 250, SLOW_SPEED = 120, NORMAL_SPEED=250;
//	private final static int FAST_SPEED = 300, SLOW_SPEED = 50, NORMAL_SPEED=250;
	private final static int ROTATION_SPEED = 100;
	private static double wheelRadius; 
	private static double wheelsDistance;
	
	// The error and threshold values
	private final static double LOW_ANGLE_BANDWIDTH_DEG = 2;
	private final static double LOW_ANGLE_BANDWIDTH_RAD = Math.toRadians(LOW_ANGLE_BANDWIDTH_DEG);
	
	private final static double HIGH_ANGLE_BANDWIDTH_DEG = 4;
	private final static double HIGH_ANGLE_BANDWIDTH_RAD = Math.toRadians(HIGH_ANGLE_BANDWIDTH_DEG);
	// Used when the robot is possibly turning fast
	
	private final static double LOW_POSITION_BANDWIDTH = 1;
	private final static double HIGH_POSITION_BANDWIDTH = 10;
	

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
	private static final int FRONT_DISTANCE_THRESHOLD = 20, ANGLED_SENSOR_DISTANCE_THRESHOLD = 18, ANGLED_SENSOR_BANDCENTER = 30;
	private static final int ANGLED_SENSOR_DISTANCE_BANDWIDTH = 5;
	private static final int MAX_DISTANCE = 40;
	private static final int UPPER_ANGLED_BANDWIDTH = 15;
	private static final int LOWER_ANGLED_BANDWIDTH = 5;
	private static double rotationSpeed;
	
	private int[] distance = {255,255,255};
	private double launchAngle = Math.tan(7.4/(5*30.48 + 4));
	private double launchDistance = Math.sqrt(Math.pow(-7.4, 2) + Math.pow(5*30.48 + 4, 2));
	private Launcher launcher;
	private int targetAngle;
	private double targetX;
	private double targetY;
	
	private int followingSide;
	private static final int P_SPEED_COEFFICIENT = 10;
	
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
		if(speed[LEFT] > FAST_SPEED){
			speed[LEFT] = FAST_SPEED;
 		}
		else if(speed[LEFT] < SLOW_SPEED){
			speed[LEFT] = SLOW_SPEED;
 		}
		
		if(speed[RIGHT] > FAST_SPEED){
			speed[RIGHT] = FAST_SPEED;
 		}
		else if(speed[RIGHT] < SLOW_SPEED){
			speed[RIGHT] = SLOW_SPEED;
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
		turnTo(targetAngle + launchAngle,true);
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
	 * This function takes as arguments the x and y position in cm Will travel to designated position, while
	 * constantly updating it's heading
	 * @param x destination x coordinate
	 * @param y destination y coordinate
	 * @param obstacleAvoidance determines if the obstacle avoidance feature is activated or not
	 */
	public void travelTo(double x, double y,boolean obstacleAvoidance) {
		destination.setX(x);
		destination.setY(y);
		double relativeTargetOrientation = minimizeAngle( (destination.subtract(position)).getOrientation() )*180/Math.PI;

		// If the robot isn't close enough to its destination
		while( !position.approxEquals(destination, LOW_POSITION_BANDWIDTH) ) {
//			this.distance[FRONT] = usController.getDistance(FRONT);
//			this.distance[LEFT] = usController.getDistance(LEFT);
//			this.distance[RIGHT] = usController.getDistance(RIGHT);
			
			this.distance[FRONT] = usController.getFilteredDistance(FRONT);
			this.distance[LEFT] = usController.getFilteredDistance(LEFT);
			this.distance[RIGHT] = usController.getFilteredDistance(RIGHT);
			
			 if( obstacleAvoidance && !position.approxEquals(destination, HIGH_POSITION_BANDWIDTH) ) {
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
				else if( distance[LEFT] <= ANGLED_SENSOR_DISTANCE_THRESHOLD ) {
					while(distance[LEFT] < ANGLED_SENSOR_BANDCENTER){
						turn(RIGHT);
						distance[LEFT] = usController.getFilteredDistance(LEFT);
					}
					avoidObstacle(LEFT);
				}
				else if( distance[RIGHT] <= ANGLED_SENSOR_DISTANCE_THRESHOLD ) {
					while(distance[RIGHT] < ANGLED_SENSOR_BANDCENTER){
						turn(LEFT);
						distance[RIGHT] = usController.getFilteredDistance(RIGHT);
					}
					avoidObstacle(RIGHT);
				}
				else {
					turnTo(relativeTargetOrientation, false);
					moveStraight();
				}
			}
			
			
//			// Obstacle avoidance while on the set path
//			else if(distance[LEFT]<=LEFT_MIN_DISTANCE){
//				moveRight();
//			}
			else
			{
				turnTo(relativeTargetOrientation, false);
				moveStraight();
			}
			updatePosition();
			relativeTargetOrientation = minimizeAngle( (destination.subtract(position)).getOrientation() )*180/Math.PI;
		}
		
		// If the robot reached its destination
		wheels[LEFT].stop();
		wheels[RIGHT].stop();
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
				move(followingSide);
//				pMove(followingSide, distError);
			}
			
			
			/* Case 2: Negative error, moving too close to wall */
			else if (distError < - LOWER_ANGLED_BANDWIDTH) {
				if(followingSide == LEFT){
					move(RIGHT);
//					pMove(RIGHT, distError);
				}
				else if(followingSide == RIGHT){
					move(LEFT);
//					pMove(LEFT, distError);
				}
			}
			
			/* Case 1:  Error in bounds  */
			else /*if (Math.abs(distError) <= ANGLED_SENSOR_DISTANCE_BANDWIDTH)*/ { 
				move(FRONT);
			}

			
		}

		
	}

	/**
	 * TurnTo function which takes an angle and boolean as arguments The boolean controls whether or not to stop the
	 * motors when the turn is completed
	 * @param targetAngle The angle in degrees
	 */
	public void turnTo(double targetAngle, boolean stop) {
		while(Math.abs(Odometer.minAngleFromTo(odometer.getHeading()*180/Math.PI,targetAngle))>LOW_ANGLE_BANDWIDTH_DEG){
			// If the robot has to turn counterclockwise
			if(Odometer.minAngleFromTo(odometer.getHeading()*180/Math.PI,targetAngle)<0){
				wheels[RIGHT].forward();
				wheels[LEFT].backward();
			}
			// Else if the robot has to turn clockwise
			else{
				wheels[RIGHT].backward();
				wheels[LEFT].forward();
			}

			speed[LEFT] = ROTATION_SPEED;
			speed[RIGHT] = ROTATION_SPEED;
			updateSpeed();
			
		}
		if( stop )
			this.stop();
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
	 * Minimizes an angle so that it is within the range from -180 degrees to 180 if not already in that range
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
		if (leftSpeed > 900.0)
			wheels[LEFT].setSpeed(900);
		else
			wheels[LEFT].setSpeed((int)leftSpeed);
		
		if (rightSpeed > 900.0)
			wheels[RIGHT].setSpeed(900);
		else
			wheels[RIGHT].setSpeed((int)rightSpeed);
	}
	
	/**
	 * Stops the robot
	 */
	public void stop(){
		wheels[RIGHT].stop();
		wheels[LEFT].stop();
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