import lejos.nxt.UltrasonicSensor;

public class USLocalizer extends Localizer {
	private UltrasonicSensor usSensor;

	public USLocalizer(Odometer odometer, Navigator navigator, UltrasonicSensor usSensor) {
		super(odometer, navigator);
		this.usSensor = usSensor;
	}
	
	@Override
	public void doLocalization() {
	}

}
