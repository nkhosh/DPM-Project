import lejos.nxt.ColorSensor;

public class LSLocalizer extends Localizer {
	private ColorSensor lsSensor;
	
	public LSLocalizer(Odometer odometer, Navigator navigator, ColorSensor lsSensor1, ColorSensor lsSensor2) {
		super(odometer, navigator);
		this.lsSensor = lsSensor;
	}
	
	@Override
	public void doLocalization() {
	}
}
