package main;
import lejos.nxt.NXTRegulatedMotor;

public class Odometer extends Thread {	
	private static final long ODOMETER_PERIOD = 25;
	private static double wheelRadius=2.055, wheelsDistance=16.8;
//	Previous values in Navigator: (Check for errors)
//	private final static double RADIUS = 2.085; 
//	private final static double WHEELS_DISTANCE = 16.2;
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
		tachometer[0] = 0;
		tachometer[1] = 0;
		this.wheels = wheels;
		wheels[0].resetTachoCount();
		wheels[1].resetTachoCount();
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
	
	public void setRadius(double r) {
		wheelRadius = r;
	}
	
	public double getRadius() {
		return wheelRadius;
	}
	
	public void setWidth(double w) {
		wheelsDistance = w;
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
	
	public static double minAngleFromTo(double fromAngle, double toAngle) {
		double d = fixDegAngle(toAngle - fromAngle);
		
		if (d < 180.0)
			return d;
		else
			return d - 360.0;
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
	
	public void getPosition(double [] pos) {
		synchronized (lock) {
			pos[0] = x;
			pos[1] = y;
			pos[2] = fixDegAngle(Math.toDegrees(heading));
		}
	}
	
	public void setPosition(double [] pos, boolean [] update) {
		synchronized (lock) {
			if (update[0]) x = pos[0];
			if (update[1]) y = pos[1];
			if (update[2]) heading = fixRadAngle(Math.toRadians(pos[2]));
		}
	}

	public boolean isTurning(){
		return !(wheels[0].getSpeed() == wheels[1].getSpeed());
	}
	
	public void run() {
		long updateStart, updateEnd;

		while (true) {
			updateStart = System.currentTimeMillis();
			double tachoCounterL = (wheels[0].getTachoCount())*Math.PI/180;
			double tachoCounterR = (wheels[1].getTachoCount())*Math.PI/180;
			tachometer[0] = tachoCounterL - tachometer[0];
			tachometer[1] = tachoCounterR - tachometer[1];
			dc = (tachometer[1]*wheelRadius + tachometer[0]*wheelRadius)/2;
			dt = (tachometer[1]*wheelRadius - tachometer[0]*wheelRadius)/wheelsDistance;
			
			synchronized (lock) {
				// don't use the variables x, y, or theta anywhere but here!
				y += dc * Math.cos(heading + dt/2);
				x += dc * Math.sin(heading + dt/2);
				heading -= dt;
				heading = fixRadAngle(heading);
			}
			
			tachometer[0] = tachoCounterL;
			tachometer[1] = tachoCounterR;
			
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