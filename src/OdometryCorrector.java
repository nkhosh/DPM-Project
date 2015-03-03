
public class OdometryCorrector extends Thread {
	Object lock;
	public OdometryCorrector(Object lock) {
		this.lock = lock;
	}
}
