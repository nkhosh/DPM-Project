package main;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.Sound;


public class Navigator extends Thread {
	// Variables for the state of the movement
	private final static boolean NO_OBSTACLE = true, OBSTACLE_AVOIDING = false;
	private boolean state;
	
	// Variables for the speed of the movement
	final static int FAST_SPEED = 200, SLOW_SPEED = 100, NORMAL_SPEED=200, ACCELERATION = 4000;
	private final static int ROTATION_SPEED = 100;
	
	// The error and threshold values
	final static double DEG_ERR = 2.0, CM_ERR = 1.0;
	private static final int THREAD_PERIOD = 15;	
	final static int POSITION_BANDWIDTH = 1;
	private static final int DISTANCE_THRESHOLD = 15;
	private static final double ANGLE_BANDWIDTH = 2*Math.PI/180; // Used when the robot is possibly turning fast
	
	// Objects used by the navigator
	private Odometer odometer;
	private NXTRegulatedMotor[] wheels;
	private int[] speed;
	private USController usController;
	
	// Variables to indicate the position of RobinHood and the destination
	private Vector position, destination;
	private Vector unitOrientationVector;
	boolean isNavigating;	
	private double[][] destinationArray;
	private int destinationIndex;
	
	// Obstacle avoidance variables
	private int obstacleAvoidanceGapCounter=0; 
	private static final int OBSTACLE_AVOIDANCE_GAP_FILTER=3, OBSTACLE_MAX_DISTANCE=100, OBSTACLE_DISTANCE_THRESHOLD=3;
	
	
	private final static int LEFT=0, FRONT=1, RIGHT=1;
	private final static int MAX_FRONT_DISTANCE = 20;
	private int[] distance;
	
	
	public Navigator(Odometer odometer, NXTRegulatedMotor[] wheels, USController usController)	{
		this.odometer = odometer;
		this.wheels = wheels;
		this.state = NO_OBSTACLE;
		this.usController = usController;
		this.speed = new int[2];
		this.speed[LEFT] = this.speed[RIGHT] = NORMAL_SPEED;
		this.isNavigating = false;
		destination = new Vector(0, 0);
		position = new Vector(0, 0);
		destinationIndex = 0;
		unitOrientationVector = new Vector();
	}
	
	/**
	 * Moves the robot straight.
	 */
	protected void moveStraight() {
		speed[LEFT]=NORMAL_SPEED;
		speed[RIGHT]=NORMAL_SPEED;
	}
	/**
	 * Turns right instantaneously when there is a close wall in front.
	 * Rotates about the center axis of the robot.
	 */
	protected void turnRight() {
		speed[LEFT]=NORMAL_SPEED;
		wheels[RIGHT].backward();
		speed[RIGHT]=NORMAL_SPEED;
		updateSpeed();
		wheels[RIGHT].forward();
	}
	/**
	 * Turns left instantaneously when there is no wall in the left.
	 * Rotates about the center axis of the robot.
	 */
	protected void turnLeft() {
		speed[RIGHT]=NORMAL_SPEED;
		wheels[LEFT].backward();
		speed[LEFT]=NORMAL_SPEED;
		updateSpeed();
		wheels[LEFT].forward();
	}	
	protected void moveLeft() {
		speed[LEFT] = SLOW_SPEED;
		speed[RIGHT] = FAST_SPEED;
	}
	protected void moveRight() {
		speed[LEFT] = FAST_SPEED;
		speed[RIGHT] = SLOW_SPEED;
	}
	protected void updateSpeed() {
		wheels[LEFT].setSpeed(speed[LEFT]);		     
		wheels[RIGHT].setSpeed(speed[RIGHT]);	
	}
	
	/**
	 * Updates the position vector and the unit orientation vector of the robot based
	 * on the odometer x, y and theta
	 */
	public void updatePosition() {
		position.setX(odometer.getX());
		position.setY(odometer.getY());
		unitOrientationVector.setOrientation(minimizeAngle(odometer.getHeading()));
	}
	public void setSpeeds(int lSpd, int rSpd) {
		this.wheels[0].setSpeed(lSpd);
		this.wheels[1].setSpeed(rSpd);
		if (lSpd < 0)
			this.wheels[0].backward();
		else
			this.wheels[0].forward();
		if (rSpd < 0)
			this.wheels[1].backward();
		else
			this.wheels[1].forward();
	}
	/**
	 * TravelTo function which takes as arguments the x and y position in cm Will travel to designated position, while
	 * constantly updating it's heading
	 */
	public void travelTo(double x, double y) {
		isNavigating = true;
		double relativeTargetOrientation = minimizeAngle( (destination.subtract(position)).getOrientation() )*180/Math.PI;
		
		// If the robot isn't close enough to its destination
		if( !position.approxEquals(destination) ) {
			if( state == OBSTACLE_AVOIDING ) {
				// If the robot is facing close enough to the destination OR it has reached the destination
				if( Math.abs(relativeTargetOrientation - (unitOrientationVector.getOrientation())) <= ANGLE_BANDWIDTH 
					|| position.approxEquals(destination)) {
					state = NO_OBSTACLE;
				}
				else {
					avoidObstacle();
				}
			}
			else if( state == NO_OBSTACLE ) {
				// If there's an obstacle directly in front of the robot
				if( distance[FRONT]<=MAX_FRONT_DISTANCE ) {
					// Keep rotating right until the sensor doesn't detect the wall
					while(usController.getDistance(FRONT) <= DISTANCE_THRESHOLD){
						wheels[RIGHT].backward();
						wheels[LEFT].forward();
						wheels[LEFT].setSpeed(ROTATION_SPEED);
						wheels[RIGHT].setSpeed(ROTATION_SPEED);
						updatePosition();
					}
					state = OBSTACLE_AVOIDING;
					wheels[RIGHT].forward();
					wheels[LEFT].forward();
				}
				// There's no obstacle in the way, turn towards the destination
				else {
					turnTo(relativeTargetOrientation, false);
					wheels[LEFT].forward();
					wheels[RIGHT].forward();
					wheels[LEFT].setSpeed(NORMAL_SPEED);
					wheels[RIGHT].setSpeed(NORMAL_SPEED);
				}
			}
			updatePosition();
			relativeTargetOrientation = minimizeAngle( (destination.subtract(position)).getOrientation() )*180/Math.PI;
		}
		
		// If the robot reached its destination
		else {
			isNavigating = false;
			wheels[LEFT].stop();
			wheels[RIGHT].stop();
			// Set the destination to the next set of coordinates in the destinationArray
			if(destinationIndex < destinationArray.length - 1 ){
				destinationIndex++;
				destination.setX(destinationArray[destinationIndex][0]);
				destination.setY(destinationArray[destinationIndex][1]);
			}
		}
	}

	/**
	 * TurnTo function which takes an angle and boolean as arguments The boolean controls whether or not to stop the
	 * motors when the turn is completed
	 * @param targetAngle The angle in degrees
	 */
	public void turnTo(double targetAngle, boolean stop) {
		
		while(Math.abs(Odometer.minAngleFromTo(odometer.getHeading()*180/Math.PI,targetAngle))>DEG_ERR){
			
			// If the robot has to turn counterclockwise
			if(Odometer.minAngleFromTo(odometer.getHeading()*180/Math.PI,targetAngle)<0){
				wheels[LEFT].backward();
				wheels[RIGHT].forward();

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
			this.setSpeeds(0, 0);
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
	/**
	 * Sets the array of coordinates to be reached.
	 * The initial destination is set to be the first element of the array.
	 * @param destinationArray
	 */
	public void setDestinationArray(double[][] destinationArray) {
		this.destinationArray = destinationArray;
		destination.setX(destinationArray[0][0]);
		destination.setY(destinationArray[0][1]);
	}
	
	/**
	 * This class controls the movement of the robot when an obstacle is detected.
	 * If there is an obstacle in front, it turns right.
	 * Otherwise it follows an obstacle with the same logic as lab 1 wall follower Patbot.
	 */
	private void avoidObstacle() {
		this.distance[FRONT] = usController.getDistance(FRONT);
		this.distance[LEFT] = usController.getDistance(LEFT);
		
		// Wall in front
		if(distance[FRONT]<MAX_FRONT_DISTANCE){
			// Turn right
			turnRight();
		}
		else{
			moveStraight();
		}
		// Sensor is to the left
		int distError = distance[LEFT]-DISTANCE_THRESHOLD;
		// Potential concave turn
		if( obstacleAvoidanceGapCounter == OBSTACLE_AVOIDANCE_GAP_FILTER ) {
			obstacleAvoidanceGapCounter = 0;
			// It's not a gap, the robot has to turn left
			// Turn left
			turnLeft();
		}
		
		distError = distance[RIGHT]-DISTANCE_THRESHOLD;
		// Potential gap to the left
		if (distance[FRONT] > OBSTACLE_MAX_DISTANCE) {
			obstacleAvoidanceGapCounter++;
			moveStraight();
		}
		// Wall to the left
		else {
			obstacleAvoidanceGapCounter = 0;
			/* Case 1:  Error in bounds  */
			if (Math.abs(distError) <= OBSTACLE_DISTANCE_THRESHOLD) {
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
		}
		updateSpeed();
	}
	
	/**
	 * Initiates the movement of Robin Hood when started. Calls the method travelTo on the specified destinations.
	 */
	public void run() {
		wheels[LEFT].forward();
		wheels[RIGHT].forward();
		wheels[LEFT].setSpeed(NORMAL_SPEED);
		wheels[RIGHT].setSpeed(NORMAL_SPEED);
		long updateEnd,updateStart;
		while(true) {
			updateStart = System.currentTimeMillis();
			
			travelTo(destination.getX(), destination.getY());
			
			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < THREAD_PERIOD) {
				try {
					Thread.sleep(THREAD_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}

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
