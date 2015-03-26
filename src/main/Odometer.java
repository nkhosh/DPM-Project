package main;

import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;

public class Odometer extends Thread {	
	private double x, y, heading;
	Object lock;
	private double radius, width;
	private final NXTRegulatedMotor[] wheels;
	private double[] tachometer;
	private double dc, dt;
	private static final long ODOMETER_PERIOD = 25;
	
	public Odometer(NXTRegulatedMotor[] wheels, Object lock) {
		this.lock = lock;
		x = 0.0;
		y = 0.0;
		heading = 0.0;
		tachometer = new double[2];
		tachometer[0] = 0;
		tachometer[1] = 0;
		this.wheels = wheels;
		wheels[0].resetTachoCount();
		wheels[1].resetTachoCount();
		
		// radius and width determined after calibration
//		radius = 2.075; 
//		width = 16.22;
		
		radius = 2.085; //without shade friction
//		width = 16.19;
		width = 16.2;
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
		radius = r;
	}
	public double getRadius() {
		return radius;
	}
	public void setWidth(double w) {
		width = w;
	}
	public double getWidth() {
		return width;
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
			return fixRadAngle(heading);
		}
	}
	public double getHeadingDeg() {
		synchronized(lock){
			return fixDegAngle(Math.toDegrees(heading));
			
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
			pos[2] = Math.toDegrees(heading);
		}
	}
	
	public void setPosition(double [] pos, boolean [] update) {
		synchronized (lock) {
			if (update[0]) x = pos[0];
			if (update[1]) y = pos[1];
			if (update[2]) heading = fixRadAngle(Math.toRadians(pos[2]));
		}
	}
	
	public void run() {
		long updateStart, updateEnd;

		while (true) {
			updateStart = System.currentTimeMillis();
			// put (some of) your odometer code here
			double tachoCounterL = (wheels[0].getTachoCount())*Math.PI/180;
			double tachoCounterR = (wheels[1].getTachoCount())*Math.PI/180;
			tachometer[0] = tachoCounterL - tachometer[0];
			tachometer[1] = tachoCounterR - tachometer[1];
			dc = (tachometer[1]*radius + tachometer[0]*radius)/2;
			dt = (tachometer[1]*radius-tachometer[0]*radius)/width;
			
			synchronized (lock) {
				// don't use the variables x, y, or theta anywhere but here!
				y += dc * Math.cos(heading + dt/2);
				x += dc * Math.sin(heading + dt/2);
				heading -= dt;
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
	

	public boolean isTurning(){
		return !(wheels[0].getSpeed() == wheels[1].getSpeed());
	}
}
