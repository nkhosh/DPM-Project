package main;

import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;

public class Odometer extends Thread {
	private double x, y, angle;
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
		angle = 0.0;
		tachometer = new double[2];
		tachometer[0] = 0;
		tachometer[1] = 0;
		this.wheels = wheels;
		wheels[0].resetTachoCount();
		wheels[1].resetTachoCount();
	}
	
	public void setX(double x) {
		this.x = x;
	}
	
	public void setY(double y) {
		this.y = y;
	}
	
	public void setAngle(double angle) {
		this.angle = angle;
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
		return x;
	}
	
	public double getY() {
		return y;
	}
	
	public double getAngle() {
		return angle;
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
				x += dc * Math.cos(angle + dt/2);
				y += dc * Math.sin(angle + dt/2);
				angle += dt;
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
