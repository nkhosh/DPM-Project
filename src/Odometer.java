
public class Odometer extends Thread {
	private double x, y, angle;
	private OdometryCorrector odometryCorrctor;
	Object lock;
	
	public Odometer(OdometryCorrector corrector, Object lock) {
		this.lock = lock;
		odometryCorrctor = corrector;
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
	
	public double getX() {
		return x;
	}
	
	public double getY() {
		return y;
	}
	
	public double getAngle() {
		return angle;
	}
	
	public double minAngleFromTo(double fromAngle, double toAngle) {
		return 0;
	}
}
