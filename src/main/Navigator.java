package main;
import lejos.nxt.LCD;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.Sound;


public class Navigator{	
	// Variables for the speed of the movement
	private final static int FAST_SPEED = 250, SLOW_SPEED = 90, NORMAL_SPEED=200, ACCELERATION=4000;
	private final static int ROTATION_SPEED = 100;
	private final static double RADIUS = 2.085; 
	private final static double WHEELS_DISTANCE = 16.2;
	
	// The error and threshold values
	private final static double ANGLE_BANDWIDTH_DEG = 2, ANGLE_BANDWIDTH_RAD = 2*Math.PI/180;
	private final static double POSITION_BANDWIDTH = 1.0; // Used when the robot is possibly turning fast
	
	// Objects used by the navigator
	private Odometer odometer;
	private NXTRegulatedMotor[] wheels;
	private int[] speed;
	private USController usController;
	
	// Variables to indicate the position of RobinHood and the destination
	private Vector position, destination;
	private Vector unitOrientationVector;
	boolean isNavigating;
	
	// Obstacle avoidance variables
	private int obstacleAvoidanceGapCounter=0; 
	private final static int OBSTACLE_AVOIDANCE_GAP_FILTER=3, OBSTACLE_MAX_DISTANCE=100, OBSTACLE_DISTANCE_THRESHOLD=3;
	private int obstaclePosition;
	private final static int MIN_FRONT_DISTANCE = 25, MIN_LEFT_DISTANCE = 5;
	private static double rotationSpeed;
	
	
	private final static int LEFT=0, FRONT=1, RIGHT=1;
	private int[] distance = {255,255};
	private double launchAngle = Math.tan(7.4/(5*30.48 + 4));
	private double launchDistance = Math.sqrt(Math.pow(-7.4, 2) + Math.pow(5*30.48 + 4, 2));
	private Launcher launcher;
	private OdometryCorrector odoC;
	public static int targetAngle;
	public static double targetX;
	public static double targetY;
	
	
	public Navigator(Odometer odometer, NXTRegulatedMotor[] wheels, USController usController, Launcher launch, OdometryCorrector odoC)	{
		this.launcher = launch;
		this.odometer = odometer;
		this.wheels = wheels;
		this.odoC = odoC;
//		this.state = NO_OBSTACLE;
		this.usController = usController;
		this.speed = new int[2];
		this.speed[LEFT] =  NORMAL_SPEED;
		this.speed[RIGHT] = NORMAL_SPEED;
		this.isNavigating = false;
		destination = new Vector(0, 0);
		position = new Vector(0, 0);
		unitOrientationVector = new Vector();
		updatePosition();
	}
	
	/**
	 * Moves the robot straight.
	 */
	protected void moveStraight() {
		wheels[LEFT].forward();
		wheels[RIGHT].forward();
//		speed[LEFT]=(int)(NORMAL_SPEED*odometer.getLeftRadius()/odometer.getRightRadius());
		speed[LEFT]=NORMAL_SPEED;
		speed[RIGHT]=NORMAL_SPEED;
		
		updateSpeed();
	}
	protected void moveLeft() {
		wheels[LEFT].forward();
		wheels[RIGHT].forward();
		speed[LEFT] = SLOW_SPEED;
		speed[RIGHT] = FAST_SPEED;
		updateSpeed();
	}
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
	protected void updateSpeed() {
//		wheels[LEFT].setAcceleration(ACCELERATION);		     
//		wheels[RIGHT].setAcceleration(ACCELERATION);
		wheels[LEFT].setSpeed(speed[LEFT]);		     
		wheels[RIGHT].setSpeed(speed[RIGHT]);	
	}
	
	/**
	 * Updates the position vector and the unit orientation vector of the robot based
	 * on the odometer x, y and theta
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
	//	Button.waitForAnyPress();
		travelTo(targetX,targetY,false);
		
		turnTo(targetAngle + launchAngle,true);
		
		launcher.launch( numBalls);
	}
	
	public void updatePosition() {
		position.setX(odometer.getX());
		position.setY(odometer.getY());
		unitOrientationVector.setOrientation(minimizeAngle(odometer.getHeading()));
	}
//	public void setSpeeds(int lSpd, int rSpd) {
//		this.wheels[0].setSpeed(lSpd);
//		this.wheels[1].setSpeed(rSpd);
//		if (lSpd < 0)
//			this.wheels[0].backward();
//		else
//			this.wheels[0].forward();
//		if (rSpd < 0)
//			this.wheels[1].backward();
//		else
//			this.wheels[1].forward();
//	}


	public void navigateMap(double[][]map) {
	
	for(int i = 0; i < map.length ; i++)
		{
			travelTo(map[i][0],map[i][1],false);
		}
	}
	
	/** 
	 * This function takes as arguments the x and y position in cm Will travel to designated position, while
	 * constantly updating it's heading
	 */
	public void travelTo(double x, double y,boolean obstacleAvoidance) {
		int minFront = 20, minLeft=5, bw=2;
		isNavigating = true;
		destination.setX(x);
		destination.setY(y);
		double relativeTargetOrientation = minimizeAngle( (destination.subtract(position)).getOrientation() )*180/Math.PI;
//		wheels[LEFT].setAcceleration(1000);
//		wheels[RIGHT].setAcceleration(1000);
		
		// If the robot isn't close enough to its destination
		while( !position.approxEquals(destination) ) {
			this.distance[FRONT] = usController.getDistance(FRONT);
			this.distance[LEFT] = usController.getDistance(LEFT);
			
			if( obstacleAvoidance ) {
				if( distance[FRONT] <= 20 ) {
					avoidObstacle();
				}
				else {
					turnTo(relativeTargetOrientation, false);
					moveStraight();
				}
			}
			// Used when going perpendicular to grid lines
			else if(odoC.getMustTurnLeft()){
				speed[LEFT] = 0;
				speed[RIGHT] = FAST_SPEED;
				updateSpeed();
			}
			else if(odoC.getMustTurnRight()){
				speed[LEFT] = FAST_SPEED;
				speed[RIGHT] = 0;
				updateSpeed();
			}
			
			// Obstacle avoidance while on the set path
			else if(distance[LEFT]<=9){
				Sound.beep();
				moveRight();
			}
			
			else
			{
				turnTo(relativeTargetOrientation, false);
				moveStraight();
			}
			updatePosition();
			relativeTargetOrientation = minimizeAngle( (destination.subtract(position)).getOrientation() )*180/Math.PI;
		}
		
		// If the robot reached its destination
		isNavigating = false;
		wheels[LEFT].stop();
		wheels[RIGHT].stop();
	}

	/**
	 * This class controls the movement of the robot when an obstacle is detected.
	 * If there is an obstacle in front, it turns right.
	 * Otherwise it follows an obstacle with the same logic as bang bang controller.
	 */
	private void avoidObstacle() {
		this.distance[FRONT] = usController.getDistance(FRONT);
		this.distance[LEFT] = usController.getDistance(LEFT);
		double relativeTargetOrientation = minimizeAngle( (destination.subtract(position)).getOrientation() );
		int distError;

		while ( true ) {
			turnRight();
			if( distance[LEFT] < 16 )
				break;
			this.distance[LEFT] = usController.getDistance(LEFT);
			LCD.drawString("Left dist:  "+distance[LEFT], 0, 4);
			LCD.drawString("Front dist: "+distance[FRONT], 0, 5);
		}
		
		while( true ) {
			distance[FRONT] = usController.getDistance(FRONT);
			distance[LEFT] = usController.getDistance(LEFT);
			updatePosition();
			LCD.drawString("Left dist:  " + distance[LEFT], 0, 4);
			LCD.drawString("Front dist: " + distance[FRONT], 0, 5);
			relativeTargetOrientation = minimizeAngle( (destination.subtract(position)).getOrientation() );
			distError = distance[LEFT]-10;
			/* Case 1:  Error in bounds  */
			if (Math.abs(distError) <= 1.5 ) {
				moveStraight();
			}
			/* Case 2: Negative error, moving too close to wall */
			else if (distError < 0) {
				moveRight();
			}
			/* Case 3: Positive error, moving too far from wall */
			else if (distError > 0) {
				moveLeft();
			}
			if( Math.abs(relativeTargetOrientation - (unitOrientationVector.getOrientation())) <= 3*Math.PI/180 && distance[FRONT]>40 ) // why?
				break;
			
			if( distance[FRONT] <= 20 ) { // why?
				break;
			}
		}
		return;
	}

	/**
	 * TurnTo function which takes an angle and boolean as arguments The boolean controls whether or not to stop the
	 * motors when the turn is completed
	 * @param targetAngle The angle in degrees
	 */
	public void turnTo(double targetAngle, boolean stop) {
//		wheels[LEFT].setAcceleration(6000);
//		wheels[RIGHT].setAcceleration(6000);

		while(Math.abs(Odometer.minAngleFromTo(odometer.getHeading()*180/Math.PI,targetAngle))>ANGLE_BANDWIDTH_DEG){
			// If the robot has to turn counterclockwise
			if(Odometer.minAngleFromTo(odometer.getHeading()*180/Math.PI,targetAngle)<0){
				wheels[RIGHT].forward();
				wheels[LEFT].backward();

				wheels[LEFT].setSpeed(ROTATION_SPEED);
				wheels[RIGHT].setSpeed(ROTATION_SPEED);
			}
			// Else if the robot has to turn clockwise
			else{
				wheels[RIGHT].backward();
				wheels[LEFT].forward();

				wheels[LEFT].setSpeed(ROTATION_SPEED);
				wheels[RIGHT].setSpeed(ROTATION_SPEED);
			}
		}
		if( stop )
			this.stop();

//		wheels[LEFT].setAcceleration(1000);
//		wheels[RIGHT].setAcceleration(1000);
	}
	
	
	public static double convertDistance(double radius, double distance) {
		return ((180.0 * distance) / (Math.PI * radius));
	}
	public static double convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	/**
	 * Minimizes an angle so that it is within the range from -180 degrees to 180 if not already in that range
	 * @param angle in radians
	 * @return
	 */
	public double minimizeAngle(double angle){
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
		
	public void setRotationSpeed(double speed) {
		rotationSpeed = speed;
		setSpeeds(rotationSpeed);
	}
	
	public void setSpeeds(double rotationalSpeed) {
		double leftSpeed, rightSpeed;

		this.rotationSpeed = rotationalSpeed; 

		leftSpeed = (  rotationalSpeed * WHEELS_DISTANCE * Math.PI / 360.0) *
				180.0 / (RADIUS * Math.PI);
		rightSpeed = (-rotationalSpeed * WHEELS_DISTANCE * Math.PI / 360.0) *
				180.0 / (RADIUS * Math.PI);

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
	void stop(){
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
		public double dotProduct(Vector v) {
			return x*v.getX()+y*v.getY();
		}
		public Vector add(Vector v) {
			return new Vector(this.x+v.getX(), this.y+v.getY());
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
		public double getMagnitude() {
			return magnitude;
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
		public boolean approxEquals(Vector v) {
			return (Math.abs(this.x-v.getX())<= POSITION_BANDWIDTH &&
					Math.abs(this.y-v.getY())<=POSITION_BANDWIDTH);
		}
	}
	
}