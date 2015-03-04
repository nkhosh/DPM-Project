import lejos.nxt.UltrasonicSensor;

public class USLocalizer extends Localizer {
	private UltrasonicSensor usSensor;

	public USLocalizer(Odometer odometer, Navigator navigator, UltrasonicSensor usSensor1, UltrasonicSensor usSensor2) {
		super(odometer, navigator);
		this.usSensor = usSensor;
	}
	
	@Override
	public void doLocalization() {
	}

}
