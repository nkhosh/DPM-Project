package main;
import lejos.nxt.NXTRegulatedMotor;


public class Navigator extends Thread {
	// Variables for the state of the movement
	private final static boolean NO_OBSTACLE = true, OBSTACLE_AVOIDING = false;
	private boolean state;
	
	// Variables for the speed of the movement
	final static int FAST = 200, SLOW = 100, NORMAL=200, ACCELERATION = 4000;
	private final static int ROTATION_SPEED = 100;
	
	// The error and threshold values
	final static double DEG_ERR = 3.0, CM_ERR = 1.0;
	final static int POSITION_BANDWIDTH = 1;
	private static final int FRONT_THRESHOLD = 15;
//	private static final double LOW_ANGLE_BANDWIDTH = 2*Math.PI/180; // Used when the robot is turning slow enough
	private static final double ANGLE_BANDWIDTH = 4*Math.PI/180; // Used when the robot is possibly turning fast
	
	// Objects used by the navigator
	private Odometer odometer;
	private NXTRegulatedMotor[] wheels;
	private int[] speed;
	private USController usController;
	
	// Variables to indicate the position of RobinHood and the destination
	private Vector position, destination;
	private Vector unitOrientationVector;
	boolean isNavigating;	
	private int[][] destinationArray;
	private int destinationIndex;
	
	private final static int LEFT=0, FRONT=1, RIGHT=1;
	private final static int MAX_FRONT_DISTANCE = 20;
	private int[] distance;
	
	
	public Navigator(Odometer odometer, NXTRegulatedMotor[] wheels, USController usController)	{
		this.odometer = odometer;
		this.wheels = wheels;
		this.state = NO_OBSTACLE;
		this.usController = usController;
		this.speed = new int[2];
		this.speed[LEFT] = this.speed[RIGHT] = NORMAL;
		this.isNavigating = false;
		position = new Vector(0, 0);
		destination = new Vector();
		unitOrientationVector = new Vector();
		
	}	
	
	/**
	 * Moves the robot straight.
	 */
	protected void moveStraight() {
		speed[LEFT]=NORMAL;
		speed[RIGHT]=NORMAL;
	}
	/**
	 * Turns right instantaneously when there is a close wall in front.
	 * Rotates about the center axis of the robot.
	 */
	protected void turnRight() {
		speed[LEFT]=NORMAL;
		wheels[RIGHT].backward();
		speed[RIGHT]=NORMAL;
		updateSpeed();
		wheels[RIGHT].forward();
	}
	/**
	 * Turns left instantaneously when there is no wall in the left.
	 * Rotates about the center axis of the robot.
	 */
	protected void turnLeft() {
		speed[RIGHT]=NORMAL;
		wheels[LEFT].backward();
		speed[LEFT]=NORMAL;
		updateSpeed();
		wheels[LEFT].forward();
	}	
	protected void moveLeft() {
		speed[LEFT] = SLOW;
		speed[RIGHT] = FAST;
	}
	protected void moveRight() {
		speed[LEFT] = FAST;
		speed[RIGHT] = SLOW;
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
//		double minAng;
//		while (Math.abs(x - odometer.getX()) > CM_ERR || Math.abs(y - odometer.getY()) > CM_ERR) {
//			minAng = (Math.atan2(x - odometer.getX(),y - odometer.getY() )) * (180.0 / Math.PI);
//			this.turnTo(minAng, false);
//			this.setSpeeds(FAST, FAST);
//		}
//		this.setSpeeds(0, 0);	
//		wheels[0].rotate(Navigator.convertDistance(odometer.getRadius(), y), true);
//		wheels[1].rotate(Navigator.convertDistance(odometer.getRadius(), y), false);
		
		isNavigating = true;
		destination.setX(x);
		destination.setY(y);
		double frontDistance = (double)usController.getDistance(FRONT);
		double relativeTargetOrientation = minimizeAngle( (destination.subtract(position)).getOrientation() );
		
		// If the robot isn't close enough to its destination
		if( !position.approxEquals(destination) ) {
			if( state == OBSTACLE_AVOIDING ) {
				// If the robot is facing close enough to the destination OR it has reached the destination
				if( Math.abs(relativeTargetOrientation - (unitOrientationVector.getOrientation())) <= ANGLE_BANDWIDTH 
					|| position.approxEquals(destination)) {
					state = NO_OBSTACLE;
				}
				else {
					avoidObstacle(); //TODO
				}
			}
			else if( state == NO_OBSTACLE ) {
				// If there's an obstacle directly in front of the robot
				if( frontDistance<=FRONT_THRESHOLD ) {
					// Keep rotating right until the sensor doesn't detect the wall
					while(usController.getDistance(FRONT) <= FRONT_THRESHOLD){
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
					turnTo((destination.subtract(position)).getOrientation());
					wheels[LEFT].forward();
					wheels[RIGHT].forward();
					wheels[LEFT].setSpeed(NORMAL);
					wheels[RIGHT].setSpeed(NORMAL);
				}
			}
			updatePosition();
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
	 */
	public void turnTo(double targetAngle) {
		
		while(Math.abs(Odometer.minAngleFromTo(odometer.getHeading(),targetAngle))>DEG_ERR){
			
			// If the robot has to turn counterclockwise
			if(Odometer.minAngleFromTo(odometer.getHeading(),targetAngle)<0){
				wheels[0].backward();
				wheels[1].forward();

				wheels[0].setSpeed(ROTATION_SPEED);
				wheels[1].setSpeed(ROTATION_SPEED);
			}
			// Else if the robot has to turn clockwise
			else{
				wheels[1].backward();
				wheels[0].forward();

				wheels[0].setSpeed(ROTATION_SPEED);
				wheels[1].setSpeed(ROTATION_SPEED);
			}
		}
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
	 * @param angle
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
	public void setDestinationArray(int[][] destinationArray) {
		this.destinationArray = destinationArray;
		destination.setX(destinationArray[0][0]);
		destination.setY(destinationArray[0][1]);
	}
	
	public void processUSData() {
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
		int gapCheck=0, GAPFILTER=1, bandCenter=0, gapThreshold=0, bandwidth=0;
		int distError = distance[LEFT]-bandCenter;
		// Potential concave turn
		if( gapCheck == GAPFILTER ) {
			gapCheck = 0;
			// It's not a gap, the robot has to turn left
			// Turn left
			turnLeft();
		}
		
		distError = distance[RIGHT]-bandCenter;
		// Potential gap to the left
		if (distance[FRONT] > gapThreshold) {
			gapCheck++;
			moveStraight();
		}
		// Wall to the left
		else {
			gapCheck = 0;
			/* Case 1:  Error in bounds  */
			if (Math.abs(distError) <= bandwidth) {
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

	
	//TODO
	public void avoidObstacle() {
		
	}
	
	//TODO
	public void run() {

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
