import lejos.nxt.ColorSensor;


public class OdometryCorrector extends Thread {
	Object lock;
	ColorSensor ls1, ls2;
	
	public OdometryCorrector(Object lock, ColorSensor lsSensor1, ColorSensor lsSensor2) {
		this.lock = lock;
	}
}
