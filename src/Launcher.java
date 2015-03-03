import lejos.nxt.NXTRegulatedMotor;

public class Launcher {
	private Odometer odometer;
	private NXTRegulatedMotor launcherMotor;
	public Launcher(Odometer odometer, NXTRegulatedMotor launcherMotor) {
		this.odometer = odometer;
		this.launcherMotor = launcherMotor;
	}
}
