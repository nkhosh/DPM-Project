package main;
import lejos.nxt.NXTRegulatedMotor;

public class Odometer extends Thread {	
	private static final long ODOMETER_PERIOD = 25;
	private final static int LEFT=0, RIGHT=1, HEADING=2;
	private static double wheelRadius=2.055, wheelsDistance=15.85/*16.05*/;
	private double x, y, heading;
	private Object lock;
	private final NXTRegulatedMotor[] wheels;
	private double[] tachometer;
	private double dc, dt;
	
	public Odometer(NXTRegulatedMotor[] wheels, Object lock) {
		this.lock = lock;
		x = 0;
		y = 0;
		heading = 0.0;
		tachometer = new double[2];
		tachometer[LEFT] = 0;
		tachometer[RIGHT] = 0;
		this.wheels = wheels;
		wheels[LEFT].resetTachoCount();
		wheels[RIGHT].resetTachoCount();
	}
	

	public void setRadius(double radius){
		wheelRadius = radius;
	}
	
	public void setWidth(double width){
		wheelsDistance = width;
	}
	
	public void setX(double x) {
		synchronized(lock){
			this.x = x;
		}
	}
	
	public void setY(double y) {
		synchronized(lock){
			this.y = y;
		}
	}
	
	public void setHeading(double heading) {
		synchronized(lock){
			this.heading = fixRadAngle(heading);
		}
	}
	
	public double getRadius() {
		return wheelRadius;
	}
	
	public double getWheelsDistance() {
		return wheelsDistance;
	}
	
	public double getX() {
		synchronized(lock){
			return x;
		}
	}
	
	public double getY() {
		synchronized(lock){
			return y;
		}
	}
	
	public double getHeading() {
		synchronized(lock){
			return heading;
		}
	}
	
	public double getHeadingDeg() {
		synchronized(lock){
			return Math.toDegrees(heading);
			
		}
	}
	
	/**
	 * Sets the angle to its equivalent value in the range of 0 to 360 degrees
	 * @param angle in degrees
	 * @return angle in range of 0 to 360 degrees
	 */
	public static double fixDegAngle(double angle) {		
		if (angle < 0.0)
			angle = 360.0 + (angle % 360.0);
		
		return angle % 360.0;
	}
	
	/**
	 * Sets the angle to its equivalent value in the range of 0 to 2PI rad
	 * @param angle in rad
	 * @return angle in range of 0 to 2PI
	 */
	public static double fixRadAngle(double angle) {		
		if (angle < 0.0)
			angle = Math.PI*2 + (angle % (Math.PI*2));
		
		return angle % (Math.PI*2);
	}
	
	/**
	 * Fill the array with position variables
	 * @param pos the array with entries x, y and heading
	 */
	public void getPosition(double [] position) {
		synchronized (lock) {
			position[LEFT] = x;
			position[RIGHT] = y;
			position[HEADING] = fixDegAngle(Math.toDegrees(heading));
		}
	}
	
	/**
	 * Sets the position of the robot
	 * @param pos array of x, y, heading
	 */
	public void setPosition(double [] position) {
		synchronized (lock) {
			x = position[LEFT];
			y = position[RIGHT];
			heading = fixRadAngle(Math.toRadians(position[HEADING]));
		}
	}

	/**
	 * Minimizes an angle so that it is within the range from -PI to PI if not already in that range
	 * @param angle in radians
	 * @return
	 */
	protected static double minimizeAngle(double angle){
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
	 * Runs the odometer thread.
	 * 
	 */
	public void run() {
		long updateStart, updateEnd;
	
		while (true) {
			updateStart = System.currentTimeMillis();
			double tachoCounterL = (wheels[LEFT].getTachoCount())*Math.PI/180;
			double tachoCounterR = (wheels[RIGHT].getTachoCount())*Math.PI/180;
			tachometer[LEFT] = tachoCounterL - tachometer[LEFT];
			tachometer[RIGHT] = tachoCounterR - tachometer[RIGHT];
			dc = (tachometer[RIGHT]*wheelRadius + tachometer[LEFT]*wheelRadius)/2;
			dt = (tachometer[RIGHT]*wheelRadius - tachometer[LEFT]*wheelRadius)/wheelsDistance;
			
			synchronized (lock) {
				y += dc * Math.cos(heading + dt/2);
				x += dc * Math.sin(heading + dt/2);
				heading -= dt;
				heading = minimizeAngle(heading);
			}
			
			tachometer[LEFT] = tachoCounterL;
			tachometer[RIGHT] = tachoCounterR;
			
			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}
}