package main;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.Sound;
import lejos.util.Delay;

/**
 * All the movements of the robot are controlled by this class.
 * Obstacle avoidance method is also implemented in this class
 * since it is used while the robot is moving towards its destination.
 * In addition, this class includes the information
 * and relevant calculations of the launching coordinates 
 * and navigation of the robot to those coordinates.
 * Then it calls the launcher object to fire at the targets.
 * @author Niloofar Khoshsiyar
 */
public class Navigator{	
	
	// Movement and navigation constants
	private final static int FAST_SPEED = 250, SLOW_SPEED = 120, NORMAL_SPEED=250, MIN_SPEED = 150, MAX_SPEED = 350;
	private final static int ROTATION_SPEED = 123;
		
	// Map constants
	private final static double TILE_LENGTH = 30.48;

	// Useful flags
	private final static int LEFT=0, RIGHT=1, FRONT=2;
		
	// Launcher constants (in centimeters and degrees)
	private static final double FIRING_OFFSET_X = -7.1;
	private static final double FIRING_OFFSET_Y = 5;
	private static final int FIRING_MEAN_DISTANCE = 5; //tiles
	private static final double LAUNCH_ANGLE = Math.toDegrees( Math.atan2(-FIRING_OFFSET_X,  (FIRING_MEAN_DISTANCE*TILE_LENGTH + FIRING_OFFSET_Y)) );
	private static final double LAUNCH_DISTANCE = Math.sqrt(Math.pow(FIRING_OFFSET_X, 2) + Math.pow(5*TILE_LENGTH + FIRING_OFFSET_Y, 2));
		
	// Obstacle avoidance thresholds and constants (in centimeters)
	private static final int FRONT_DISTANCE_THRESHOLD = 20, ANGLED_SENSOR_BANDCENTER = 24; // 22 
	private static final int VERY_CLOSE_THRESHOLD = 10;
	private static final int FRONT_CORNER_DISTANCE_THRESHOLD = 25;
	private static final int MAX_FRONT_DISTANCE = 30;
	private static final int ANGLED_SENSOR_DISTANCE_BANDWIDTH = 1;	
	private final static double LOW_ANGLE_BANDWIDTH_RAD = Math.toRadians(3);
	private final static double HIGH_ANGLE_BANDWIDTH_RAD = Math.toRadians(4);
	private final static double LOW_POSITION_BANDWIDTH = 1;
	private final static double HIGH_POSITION_BANDWIDTH = 30;
	private static final int P_INNER_WHEEL_SPEED_COEFFICIENT = 9;
	private static final int P_OUTER_WHEEL_SPEED_COEFFICIENT = 10;

	// Robot properties
	private static double wheelRadius, wheelsDistance;

	// Variables to indicate the position of RobinHood and the destination
	private Vector position, destination;
	private Vector unitOrientationVector;
	
	// Objects used by the navigator
	private Odometer odometer;
	private NXTRegulatedMotor[] wheelsMotor;
	private int[] wheelSpeed;
	private USController usController;
	

	//Launcher variables
	private Launcher launcher;

	// Obstacle avoidance variables
	private int followingSide;
	private int oppositeSide;
	private int[] usDistance = {255,255,255};
	
	
	public Navigator(Odometer odometer, NXTRegulatedMotor[] wheels, USController usController, Launcher launcher)	{
		this.launcher = launcher;
		this.odometer = odometer;
		this.wheelsMotor = wheels;
		this.usController = usController;
		this.wheelSpeed = new int[2];
		this.wheelSpeed[LEFT] =  NORMAL_SPEED;
		this.wheelSpeed[RIGHT] = NORMAL_SPEED;
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
	
	/**
	 * @return tile length of the map
	 */
	public double getTileLength() {
		return TILE_LENGTH;
	}
	/**
	 * Sets the side to follow the wall
	 * @param side of the robot
	 */
	public void setUSFollowingSide(int side){
		this.followingSide = side;
	}
	
	/**
	 * Turns the robot to the input orientation. The robot turns about its center.
	 * @param orientation in which the robot will turn
	 */
	protected void turn(int orientation){

		if(orientation == LEFT){
			wheelsMotor[LEFT].backward();
			wheelsMotor[RIGHT].forward();
		}
		else if(orientation == RIGHT){
			wheelsMotor[LEFT].forward();
			wheelsMotor[RIGHT].backward();
		}
		wheelSpeed[RIGHT]=NORMAL_SPEED;
		wheelSpeed[LEFT]=NORMAL_SPEED;
		updateSpeed();
	}
	
	/**
	 * Moves the robot to the input orientation.
	 * @param orientation in which the robot should move
	 */
	protected void move(int orientation){
		wheelsMotor[LEFT].forward();
		wheelsMotor[RIGHT].forward();
		
		if(orientation == LEFT){
			wheelSpeed[LEFT]=SLOW_SPEED;
			wheelSpeed[RIGHT]=FAST_SPEED;
		}
		else if(orientation == RIGHT){
			wheelSpeed[LEFT]=FAST_SPEED;
			wheelSpeed[RIGHT]=SLOW_SPEED;
		}
		else if(orientation == FRONT){
			wheelSpeed[LEFT]=NORMAL_SPEED;
			wheelSpeed[RIGHT]=NORMAL_SPEED;
		}
		updateSpeed();
	}
	
	/**
	 * Moves the robot based on p-controller.
	 * The speed of movement depends on the distance from the wall or obstacle it is following.
	 * @param orientation to which the robot should move
	 * @param difference of the robot's distance to the wall/obstacle from the band center
	 */
	protected void pMove(int orientation, int difference){

		wheelsMotor[LEFT].forward();
		wheelsMotor[RIGHT].forward();
		
		if(orientation == LEFT){
			wheelSpeed[LEFT]=NORMAL_SPEED - pInnerAddedSpeed(difference);
			wheelSpeed[RIGHT]=NORMAL_SPEED + pOuterAddedSpeed(difference);
		}
		else if(orientation == RIGHT){
			wheelSpeed[LEFT]=NORMAL_SPEED + pOuterAddedSpeed(difference);
			wheelSpeed[RIGHT]=NORMAL_SPEED - pInnerAddedSpeed(difference);
		}
		else if(orientation == FRONT){
			wheelSpeed[LEFT]=NORMAL_SPEED;
			wheelSpeed[RIGHT]=NORMAL_SPEED;
		}
		updateSpeed();
	}
	
	/**
	 * Calculates speed of the robot based on p-controller method.
	 * @param difference of the robot's distance to the wall/obstacle from the band center
	 * @return the speed based on the difference value
	 */
	private int pInnerAddedSpeed(int difference){
		return Math.abs(difference)*P_INNER_WHEEL_SPEED_COEFFICIENT;
	}
	
	/**
	 * Calculates speed of the robot based on p-controller method.
	 * @param difference of the robot's distance to the wall/obstacle from the band center
	 * @return the speed based on the difference value
	 */
	private int pOuterAddedSpeed(int difference){
		return Math.abs(difference)*P_OUTER_WHEEL_SPEED_COEFFICIENT;
	}
	
	/**
	 * Sets the speed of the motors to the current values stored in the speed array.
	 */
	private void updateSpeed() {
		if(wheelSpeed[LEFT] > MAX_SPEED){
			wheelSpeed[LEFT] = MAX_SPEED;
 		}
		else if(wheelSpeed[LEFT] < MIN_SPEED){
			wheelSpeed[LEFT] = MIN_SPEED;
 		}
		
		if(wheelSpeed[RIGHT] > MAX_SPEED){
			wheelSpeed[RIGHT] = MAX_SPEED;
 		}
		else if(wheelSpeed[RIGHT] < MIN_SPEED){
			wheelSpeed[RIGHT] = MIN_SPEED;
 		}
		
		wheelsMotor[LEFT].setSpeed(wheelSpeed[LEFT]);		     
		wheelsMotor[RIGHT].setSpeed(wheelSpeed[RIGHT]);	
	}
	
	/**
	 * Stops the robot
	 */
	public void stop(){
		wheelsMotor[RIGHT].stop();
		wheelsMotor[LEFT].stop();
	}
	
	/**
	 * Updates the current position by odometer values
	 */
	private void updatePosition() {
		position.setX(odometer.getX());
		position.setY(odometer.getY());
		unitOrientationVector.setOrientation(odometer.getHeading());
	}
	
	/**
	 * TurnTo function which takes an angle and boolean as arguments The boolean controls whether or not to stop the
	 * motors when the turn is completed
	 * @param targetAngle The angle in degrees
	 */
	public void turnToDeg(double targetAngle, boolean stop) {
		turnToRad( Math.toRadians(targetAngle), stop);
	}
	
	/**
	 * TurnTo function which takes an angle and boolean as arguments The boolean controls whether or not to stop the
	 * motors when the turn is completed
	 * @param targetAngle The angle in degrees
	 */
	public void turnToRad(double targetAngle, boolean stop) {
		double heading = odometer.getHeading();
		
		while(Math.abs( Odometer.minimizeAngle(targetAngle-heading) ) > LOW_ANGLE_BANDWIDTH_RAD){

			wheelSpeed[LEFT] = ROTATION_SPEED;
			wheelSpeed[RIGHT] = ROTATION_SPEED;
			wheelsMotor[LEFT].setSpeed(wheelSpeed[LEFT]);		     
			wheelsMotor[RIGHT].setSpeed(wheelSpeed[RIGHT]);
			
			// If the robot has to turn counterclockwise
			if( Odometer.minimizeAngle(targetAngle-heading) < 0){
				wheelsMotor[LEFT].backward();
				wheelsMotor[RIGHT].forward();

			}
			// Else if the robot has to turn clockwise
			else{
				wheelsMotor[LEFT].forward();
				wheelsMotor[RIGHT].backward();
			}
			

			
			heading = odometer.getHeading();
			
		}
		if( stop )
			this.stop();	
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
		double relativeTargetOrientation = destination.subtract(position).getOrientation();
		
		
		// If the robot isn't close enough to its destination
		while( !position.approxEquals(destination, LOW_POSITION_BANDWIDTH) ) {
			updateFollowingSide();
			this.usDistance[FRONT] = usController.getFilteredDistance(FRONT);
			this.usDistance[LEFT] = usController.getFilteredDistance(LEFT);
			this.usDistance[RIGHT] = usController.getFilteredDistance(RIGHT);
			
			if(obstacleAvoidance && !position.approxEquals(destination, HIGH_POSITION_BANDWIDTH ) ) {			
				
				// Obstacle to the left
				if( usDistance[LEFT] <= VERY_CLOSE_THRESHOLD ) {
					do{
						turn(RIGHT);
						usDistance[LEFT] = usController.getFilteredDistance(LEFT);
						usDistance[FRONT] = usController.getFilteredDistance(FRONT);
					}while(usDistance[LEFT] <= VERY_CLOSE_THRESHOLD || usDistance[FRONT] <= MAX_FRONT_DISTANCE);
					avoidObstacle();
				}
				
				// Obstacle to the right
				else if( usDistance[RIGHT] <= VERY_CLOSE_THRESHOLD ) {
					do{
						turn(LEFT);
						usDistance[FRONT] = usController.getFilteredDistance(FRONT);
						usDistance[RIGHT] = usController.getFilteredDistance(RIGHT);
					}while(usDistance[RIGHT] <= VERY_CLOSE_THRESHOLD || usDistance[FRONT] <= MAX_FRONT_DISTANCE);
					avoidObstacle();
				}
				
				
				// Obstacle in front
				else if( usDistance[FRONT] <= FRONT_DISTANCE_THRESHOLD ) {
					// Keep rotating until no obstacle in front
						while(usDistance[FRONT] <= MAX_FRONT_DISTANCE){
							if(followingSide == LEFT){
								turn(LEFT);
							}
							else{
								turn(RIGHT);
							}
							usDistance[FRONT] = usController.getFilteredDistance(FRONT);
						}
						avoidObstacle();
					}

				// Corner in front (Special case where distance gets to around 23-24 centimeters and then reads 255
				else if( usDistance[FRONT] <= FRONT_CORNER_DISTANCE_THRESHOLD && usDistance[FRONT] > FRONT_DISTANCE_THRESHOLD ) {
					Delay.msDelay(200);
					usDistance[FRONT] = usController.getFilteredDistance(FRONT);
					if(usDistance[FRONT] > MAX_FRONT_DISTANCE){
						Sound.beep();
						
						turn(followingSide);
						
						Delay.msDelay(1500);

						avoidObstacle();
					}
				}
				
				else if (usDistance[LEFT] < ANGLED_SENSOR_BANDCENTER){
					avoidObstacle();
				}
				
				else if (usDistance[RIGHT] < ANGLED_SENSOR_BANDCENTER){
					avoidObstacle();
				}
				
				// No obstacles
				else {
					turnToRad(relativeTargetOrientation, false);
					move(FRONT);
				}
			}
			// No obstacle avoidance or reaching the destination 
			else
			{
				turnToRad(relativeTargetOrientation, false);
				move(FRONT);
			}
			 
			relativeTargetOrientation = destination.subtract(position).getOrientation();
			updatePosition();
		}
		
		// If the robot reached its destination
		wheelsMotor[LEFT].stop();
		wheelsMotor[RIGHT].stop();
	}
	
	/**
	 * Makes the robot travel to the specified coordinates of the tiles.
	 * @param x destination x tile number
	 * @param y destination y tile number
	 * @param obstacleAvoidance determines if the obstacle avoidance feature is activated or not
	 */
	public void travelToTiles(double x, double y, boolean obstacleAvoidance){
		travelTo(x*TILE_LENGTH,y*TILE_LENGTH,obstacleAvoidance);
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
	 * @param map The list of x and y tile numbers of the destinations in the given map
	 */
	public void navigateMapTiles(double[][]map, boolean obstacleAvoidance) {
		for(int i = 0; i < map.length ; i++)
		{
			travelTo(map[i][0]*TILE_LENGTH,map[i][1]*TILE_LENGTH, obstacleAvoidance);
		}
	}
	
	/**
	 * Set the wall following side of the robot. This method is used for obstacle avoidance.
	 */
	private void updateFollowingSide(){
		double relativeTargetOrientation = destination.subtract(position).getOrientation();
		double heading = odometer.getHeading();
		
		// The angle between the destination vector and the current heading of the robot
		double angleOffset = Odometer.minimizeAngle(heading - relativeTargetOrientation);
		
		if(angleOffset < 0){
			followingSide = RIGHT;
			oppositeSide = LEFT;
		}
		else{
			followingSide = LEFT;
			oppositeSide = RIGHT;
		}
	}
	
	/**
	 * Moves the robot to the point that it has to launch the ball.
	 * It stops and turns to the target. Then calls the launcher to launch a ball.
	 * All the input arguments of this class are in centimeters.
	 * @param x the x coordinate of the launching point
	 * @param y the y coordinate of the launching point
	 * @param xMin the minimum possible x in the allowed launching zone 
	 * @param xMax the maximum possible x in the allowed launching zone
	 * @param yMin the minimum possible y in the allowed launching zone
	 * @param yMax the maximum possible y in the allowed launching zone
	 * @param numBalls Number of ping pong balls to shoot
	 */
	public void fireAt(double x, double y, double xMin, double xMax, double yMin, double yMax,int numBalls)
	{
		int targetAngle;
		targetAngle = -1;
		double targetX = 0;
	    double targetY = 0;
		for (int i = 0; i<270 ; i++)
		{
			double testX = (Math.cos(Math.toRadians(i+90))*LAUNCH_DISTANCE);
			double testY = (Math.sin(Math.toRadians(i+90))*LAUNCH_DISTANCE);
			
			testX = testX + x;
			testY = testY + y;
			if((testX > xMin && testX < xMax)&&(testY > yMin && testY < yMax))
			{
				targetX = testX;
				targetY = testY;
				targetAngle = 180-i;
				break;
			}
		}
		travelTo(targetX,targetY,false);
		turnToDeg(targetAngle + LAUNCH_ANGLE,true);
		launcher.launch( numBalls);
	}
	
	/**
	 * Moves the robot to the point that it has to launch the ball.
	 * It stops and turns to the target. Then calls the launcher to launch a ball.
	 * All the input arguments of this class are in tiles
	 * @param x of the target
	 * @param y of the target
	 * @param xMin of firing zone
	 * @param xMax of firing zone
	 * @param yMin of firing zone
	 * @param yMax of firing zone
	 * @param numBalls Number of ping pong balls to shoot
	 */
	public void fireAtTiles(double x, double y, double xMin, double xMax, double yMin, double yMax,int numBalls)
	{
		fireAt(x*TILE_LENGTH, y*TILE_LENGTH, xMin*TILE_LENGTH, xMax*TILE_LENGTH, yMin*TILE_LENGTH, yMax*TILE_LENGTH, numBalls);
	}
	
	/**
	 * Implementation of obstacle avoidance based on p-controller method.
	 */
	private void avoidObstacle(){
		updatePosition();
		double relativeTargetOrientation = destination.subtract(position).getOrientation();
		double heading = odometer.getHeading();
		int followingDistanceError;
		int oppositeDistanceError;
		
		while(true){
			// Obstacles on left and right, pick following side based on destination
			updateFollowingSide();
			
			// Sets the opposite side as the side that is not following the wall
			oppositeSide = (followingSide + 1) % 2;
			
			usDistance[followingSide] = usController.getFilteredDistance(followingSide);
			usDistance[oppositeSide] = usController.getFilteredDistance(oppositeSide);
			usDistance[FRONT] = usController.getFilteredDistance(FRONT);
			
			followingDistanceError = usDistance[followingSide]-ANGLED_SENSOR_BANDCENTER;
			oppositeDistanceError = usDistance[oppositeSide]-ANGLED_SENSOR_BANDCENTER;
			
			// If the way is clear ahead of the robot (including close left/right obstacles)
			if( Math.abs(relativeTargetOrientation - heading ) <= HIGH_ANGLE_BANDWIDTH_RAD 
					 &&  usDistance[followingSide] > ANGLED_SENSOR_BANDCENTER
					 && usDistance[oppositeSide] > ANGLED_SENSOR_BANDCENTER
					 &&  usDistance[FRONT] > MAX_FRONT_DISTANCE ){
				break;
			}
			// If the robot is close enough to the destination
			else if( position.approxEquals(destination, HIGH_POSITION_BANDWIDTH) ) {
				break;
			}
			// Obstacle in front, go into front avoidance
			else if(usDistance[FRONT] < FRONT_DISTANCE_THRESHOLD){
				break;
			}
			
			// Obstacle very close on following side
			else if( usDistance[followingSide] <= VERY_CLOSE_THRESHOLD ) {
				break;
			}
			
			// Obstacle very close on opposite side
			else if( usDistance[oppositeSide] <= VERY_CLOSE_THRESHOLD ) {
				break;
			}
			
			// Obstacle within bandwidth on opposite side
			else if (oppositeDistanceError < -ANGLED_SENSOR_DISTANCE_BANDWIDTH /*-LOWER_ANGLED_DISTANCE_BANDWIDTH*/) {
				if(oppositeDistanceError == LEFT){
					pMove(RIGHT, oppositeDistanceError);
				}
				else if(oppositeDistanceError == RIGHT){
					pMove(LEFT, oppositeDistanceError);
				}
			}
			
			// Case 1: Positive error, moving too far from wall
			if (followingDistanceError > ANGLED_SENSOR_DISTANCE_BANDWIDTH /*UPPER_ANGLED_DISTANCE_BANDWIDTH*/) { // so the robot doesn't turn into the wall as much
				pMove(followingSide, followingDistanceError);
			}
			// Case 2: Negative error, moving too close to wall
			else if (followingDistanceError < -ANGLED_SENSOR_DISTANCE_BANDWIDTH /*-LOWER_ANGLED_DISTANCE_BANDWIDTH*/) {
				if(followingSide == LEFT){
					pMove(RIGHT, followingDistanceError);
				}
				else if(followingSide == RIGHT){
					pMove(LEFT, followingDistanceError);
				}
			}
			// Case 3:  Error in bounds
			else { 
				move(FRONT);
			}
			
			//updates the variables
			updatePosition();
			relativeTargetOrientation = (destination.subtract(position)).getOrientation();
			heading = odometer.getHeading();
		}
	}

	/**
	 * Converts the traveling distance to the angle that the wheel has to rotate in degrees.
	 * @param radius of the wheel
	 * @param distance that must be traveled
	 * @return the angle that wheels should turn
	 */
	public static double convertDistanceToAngle(double radius, double distance) {
		return ((180.0 * distance) / (Math.PI * radius));
	}
	
	/**
	 * Converts angle that the robot should turn into the angle that the wheels have to turn in degrees.
	 * @param radius of the wheel
	 * @param wheelDistance distance of the two wheels of the robot
	 * @param angle that we want the robot to change its heading to
	 * @return the angle that wheels should turn
	 */
	public static double convertHeadingToWheelAngle(double radius, double wheelDistance, double angle) {
		return convertDistanceToAngle(radius, Math.PI * wheelDistance * angle / 360.0);
	}

	/**
	 * Sets the speed of each wheel according to the desired speed for the rotation of the robot
	 * @param headingRotationSpeed The speed at which we want the robot to change its heading
	 */
	public void setHeadingRotationSpeed(double headingRotationSpeed) {
		
		wheelSpeed[LEFT] = (int) ((  headingRotationSpeed * wheelsDistance * Math.PI / 360.0) *
				180.0 / (wheelRadius * Math.PI));
		wheelSpeed[RIGHT] = (int) ((-headingRotationSpeed * wheelsDistance * Math.PI / 360.0) *
				180.0 / (wheelRadius * Math.PI));

		// set motor directions
		if (wheelSpeed[LEFT] > 0.0)
			wheelsMotor[LEFT].forward();
		else {
			wheelsMotor[LEFT].backward();
		}
		
		if (wheelSpeed[RIGHT] > 0.0)
			wheelsMotor[RIGHT].forward();
		else 
			wheelsMotor[RIGHT].backward();
		
		updateSpeed();
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
		 * @return true if the current Vector object's x and y coordinates
		 * are approximately equal to those of the parameter Vector v, false otherwise
		 */
		public boolean approxEquals(Vector v, double bandwidth) {
			return (Math.abs(this.x-v.getX()) <= bandwidth &&
					Math.abs(this.y-v.getY()) <= bandwidth);
		}
	}
	
}