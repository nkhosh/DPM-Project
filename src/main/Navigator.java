package main;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.Sound;
import lejos.util.Delay;


public class Navigator{	
	// Variables for the speed of the movement
	private final static int FAST_SPEED = 250, SLOW_SPEED = 120, NORMAL_SPEED=250, MIN_SPEED = 150, MAX_SPEED = 350;
	private final static int ROTATION_SPEED = 123;
	private static double wheelRadius; 
	private static double wheelsDistance;
	
	// The error and threshold values
	private final static double TILE_LENGTH = 30.48;
	
	private final static double FIRING_ANGLE_BANDWIDTH_DEG = 1;
	
	private final static double LOW_ANGLE_BANDWIDTH_DEG = 3;
	private final static double LOW_ANGLE_BANDWIDTH_RAD = Math.toRadians(LOW_ANGLE_BANDWIDTH_DEG);
	
	private final static double HIGH_ANGLE_BANDWIDTH_DEG = 4;
	private final static double HIGH_ANGLE_BANDWIDTH_RAD = Math.toRadians(HIGH_ANGLE_BANDWIDTH_DEG);
	
	
	private final static double LOW_POSITION_BANDWIDTH = 1;
	private final static double HIGH_POSITION_BANDWIDTH = 30;
	

	// Obstacle avoidance variables (in centimeters)
	private static final int FRONT_DISTANCE_THRESHOLD = 20, ANGLED_SENSOR_BANDCENTER = 24; // 22 
	private static final int VERY_CLOSE_THRESHOLD = 10;
	private static final int FRONT_CORNER_DISTANCE_THRESHOLD = 25;
	private static final int MAX_FRONT_DISTANCE = 30;
	private static final int ANGLED_SENSOR_DISTANCE_BANDWIDTH = 1;
	
	// Useful flags
	private final static int LEFT=0, RIGHT=1, FRONT=2;
	
	// Objects used by the navigator
	private Odometer odometer;
	private NXTRegulatedMotor[] wheels;
	private int[] speed;
	private USController usController;
	
	// Variables to indicate the position of RobinHood and the destination
	private Vector position, destination;
	private Vector unitOrientationVector;
	
	
	private int[] distance = {255,255,255};
	private double xMean = -7.1;
	private double yMean = 5;
	
	//private double launchAngle = Math.tan(xMean/(5*30.48 + yMean));
	private double launchAngle = Math.toDegrees( Math.atan2(-xMean,  (5*30.48 + yMean)) );
//	private double launchDistance = Math.sqrt(Math.pow(-7.4, 2) + Math.pow(5*30.48 + 4, 2));
	private double launchDistance = Math.sqrt(Math.pow(xMean, 2) + Math.pow(5*30.48 + yMean, 2));
	private Launcher launcher;
	private double targetX;
	private double targetY;
	
	private int followingSide;
	private int oppositeSide;
	private static final int P_INNER_WHEEL_SPEED_COEFFICIENT = 9; // 5
	private static final int P_OUTER_WHEEL_SPEED_COEFFICIENT = 10; // 6
	
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
			wheels[LEFT].backward();
			wheels[RIGHT].forward();
		}
		else if(orientation == RIGHT){
			wheels[LEFT].forward();
			wheels[RIGHT].backward();
		}
		speed[RIGHT]=NORMAL_SPEED;
		speed[LEFT]=NORMAL_SPEED;
		updateSpeed();
	}
	
	/**
	 * Moves the robot to the input orientation.
	 * @param orientation in which the robot should move
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
	
	/**
	 * Moves the robot based on p-controller.
	 * The speed of movement depends on the distance from the wall or obstacle it is following.
	 * @param orientation to which the robot should move
	 * @param difference of the robot's distance to the wall/obstacle from the band center
	 */
	protected void pMove(int orientation, int difference){

		wheels[LEFT].forward();
		wheels[RIGHT].forward();
		
		if(orientation == LEFT){
			speed[LEFT]=NORMAL_SPEED - pInnerAddedSpeed(difference);
			speed[RIGHT]=NORMAL_SPEED + pOuterAddedSpeed(difference);
		}
		else if(orientation == RIGHT){
			speed[LEFT]=NORMAL_SPEED + pOuterAddedSpeed(difference);
			speed[RIGHT]=NORMAL_SPEED - pInnerAddedSpeed(difference);
		}
		else if(orientation == FRONT){
			speed[LEFT]=NORMAL_SPEED;
			speed[RIGHT]=NORMAL_SPEED;
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
	public void fireAtTiles(double x, double y, double xMin, double xMax, double yMin, double yMax,int numBalls)
	{
		fireAt(x*TILE_LENGTH, y*TILE_LENGTH, xMin*TILE_LENGTH, xMax*TILE_LENGTH, yMin*TILE_LENGTH, yMax*TILE_LENGTH, numBalls);
	}
	
	
	/**
	 * Stops the robot
	 */
	public void stop(){
		wheels[RIGHT].stop();
		wheels[LEFT].stop();
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

			speed[LEFT] = ROTATION_SPEED;
			speed[RIGHT] = ROTATION_SPEED;
			wheels[LEFT].setSpeed(speed[LEFT]);		     
			wheels[RIGHT].setSpeed(speed[RIGHT]);
			
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
			this.distance[FRONT] = usController.getFilteredDistance(FRONT);
			this.distance[LEFT] = usController.getFilteredDistance(LEFT);
			this.distance[RIGHT] = usController.getFilteredDistance(RIGHT);
			
			if(obstacleAvoidance && !position.approxEquals(destination, HIGH_POSITION_BANDWIDTH ) ) {			
				
				// Obstacle to the left
				if( distance[LEFT] <= VERY_CLOSE_THRESHOLD ) {
					do{
						turn(RIGHT);
						distance[LEFT] = usController.getFilteredDistance(LEFT);
						distance[FRONT] = usController.getFilteredDistance(FRONT);
					}while(distance[LEFT] <= VERY_CLOSE_THRESHOLD || distance[FRONT] <= MAX_FRONT_DISTANCE);
					avoidObstacle();
				}
				
				// Obstacle to the right
				else if( distance[RIGHT] <= VERY_CLOSE_THRESHOLD ) {
					do{
						turn(LEFT);
						distance[FRONT] = usController.getFilteredDistance(FRONT);
						distance[RIGHT] = usController.getFilteredDistance(RIGHT);
					}while(distance[RIGHT] <= VERY_CLOSE_THRESHOLD || distance[FRONT] <= MAX_FRONT_DISTANCE);
					avoidObstacle();
				}
				
				
				// Obstacle in front
				else if( distance[FRONT] <= FRONT_DISTANCE_THRESHOLD ) {
					// Keep rotating until no obstacle in front
						while(distance[FRONT] <= MAX_FRONT_DISTANCE){
							if(followingSide == LEFT){
								turn(LEFT);
							}
							else{
								turn(RIGHT);
							}
							distance[FRONT] = usController.getFilteredDistance(FRONT);
						}
						avoidObstacle();
					}

				// Corner in front (Special case where distance gets to around 23-24 centimeters and then reads 255
				else if( distance[FRONT] <= FRONT_CORNER_DISTANCE_THRESHOLD && distance[FRONT] > FRONT_DISTANCE_THRESHOLD ) {
					Delay.msDelay(200);
					distance[FRONT] = usController.getFilteredDistance(FRONT);
					if(distance[FRONT] > MAX_FRONT_DISTANCE){
						Sound.beep();
						
						turn(followingSide);
						
						Delay.msDelay(1500);

						avoidObstacle();
					}
				}
				
				else if (distance[LEFT] < ANGLED_SENSOR_BANDCENTER){
					avoidObstacle();
				}
				
				else if (distance[RIGHT] < ANGLED_SENSOR_BANDCENTER){
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
		wheels[LEFT].stop();
		wheels[RIGHT].stop();
	}
	
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
		double angleOffset = minimizeAngle(heading - relativeTargetOrientation);
		
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
	 * @param x of the target
	 * @param y of the target
	 * @param xMin of firing zone
	 * @param xMax of firing zone
	 * @param yMin of firing zone
	 * @param yMax of firing zone
	 * @param numBalls Number of ping pong balls to shoot
	 */
	public void fireAt(double x, double y, double xMin, double xMax, double yMin, double yMax,int numBalls)
	{
		int targetAngle;
		targetAngle = -1;
		targetX = 0;
	    targetY = 0;
		for (int i = 0; i<270 ; i++)
		{
			double testX = (Math.cos(Math.toRadians(i+90))*launchDistance);
			double testY = (Math.sin(Math.toRadians(i+90))*launchDistance);
			
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
		turnToDeg(targetAngle + launchAngle,true);
		launcher.launch( numBalls);
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
			
			distance[followingSide] = usController.getFilteredDistance(followingSide);
			distance[oppositeSide] = usController.getFilteredDistance(oppositeSide);
			distance[FRONT] = usController.getFilteredDistance(FRONT);
			
			followingDistanceError = distance[followingSide]-ANGLED_SENSOR_BANDCENTER;
			oppositeDistanceError = distance[oppositeSide]-ANGLED_SENSOR_BANDCENTER;
			
			// If the way is clear ahead of the robot (including close left/right obstacles)
			if( Math.abs(relativeTargetOrientation - heading ) <= HIGH_ANGLE_BANDWIDTH_RAD 
					 &&  distance[followingSide] > ANGLED_SENSOR_BANDCENTER
					 && distance[oppositeSide] > ANGLED_SENSOR_BANDCENTER
					 &&  distance[FRONT] > MAX_FRONT_DISTANCE ){
				break;
			}
			// If the robot is close enough to the destination
			else if( position.approxEquals(destination, HIGH_POSITION_BANDWIDTH) ) {
				break;
			}
			// Obstacle in front, go into front avoidance
			else if(distance[FRONT] < FRONT_DISTANCE_THRESHOLD){
				break;
			}
			
			// Obstacle very close on following side
			else if( distance[followingSide] <= VERY_CLOSE_THRESHOLD ) {
				break;
			}
			
			// Obstacle very close on opposite side
			else if( distance[oppositeSide] <= VERY_CLOSE_THRESHOLD ) {
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
	 * Minimizes an angle so that it is within the range from -PI to PI if not already in that range
	 * @param angle in radians
	 * @return the equivalent angle in -pi and pi range
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
	 * Sets the speed of each wheel according to the desired speed for the rotation of the robot
	 * @param headingRotationSpeed The speed at which we want the robot to change its heading
	 */
	public void setHeadingRotationSpeed(double headingRotationSpeed) {
		
		speed[LEFT] = (int) ((  headingRotationSpeed * wheelsDistance * Math.PI / 360.0) *
				180.0 / (wheelRadius * Math.PI));
		speed[RIGHT] = (int) ((-headingRotationSpeed * wheelsDistance * Math.PI / 360.0) *
				180.0 / (wheelRadius * Math.PI));

		// set motor directions
		if (speed[LEFT] > 0.0)
			wheels[LEFT].forward();
		else {
			wheels[LEFT].backward();
		}
		
		if (speed[RIGHT] > 0.0)
			wheels[RIGHT].forward();
		else 
			wheels[RIGHT].backward();
		
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