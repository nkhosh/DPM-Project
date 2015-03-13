package main;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.UltrasonicSensor;


public class Navigator {
	private enum State {NO_OBSTACLE, OBSTACLE_AVOIDANCE};
	final static int FAST = 200, SLOW = 100, ACCELERATION = 4000;
	private static final int ROTATION_SPEED = 100;
	final static double DEG_ERR = 3.0, CM_ERR = 1.0;
	private Odometer odometer;
	private NXTRegulatedMotor[] wheels;
	private State state;
	private USController usController;
	
	public Navigator(Odometer odometer, NXTRegulatedMotor[] wheels, USController usController)	{
		this.odometer = odometer;
		this.wheels = wheels;
		this.state = State.NO_OBSTACLE;
		this.usController = usController;
	}
	
	public void processUSData(UltrasonicSensor us) {
		
	}
	
	/*
	 * TravelTo function which takes as arguments the x and y position in cm Will travel to designated position, while
	 * constantly updating it's heading
	 */
	public void travelTo(double x, double y) {
		double minAng;
		while (Math.abs(x - odometer.getX()) > CM_ERR || Math.abs(y - odometer.getY()) > CM_ERR) {
			minAng = (Math.atan2(x - odometer.getX(),y - odometer.getY() )) * (180.0 / Math.PI);
			this.turnTo(minAng, false);
			this.setSpeeds(FAST, FAST);
		}
		this.setSpeeds(0, 0);	
//		wheels[0].rotate(Navigator.convertDistance(odometer.getRadius(), y), true);
//		wheels[1].rotate(Navigator.convertDistance(odometer.getRadius(), y), false);
	}

	/*
	 * TurnTo function which takes an angle and boolean as arguments The boolean controls whether or not to stop the
	 * motors when the turn is completed
	 */
	public void turnTo(double targetAngle, boolean stop) {
		
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
		if (stop) {
			this.setSpeeds(0, 0);
		}
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

	public static double convertDistance(double radius, double distance) {
		return ((180.0 * distance) / (Math.PI * radius));
	}

	public static double convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
}
