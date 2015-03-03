import lejos.nxt.ColorSensor;

public class LSLocalizer extends Localizer {
	private ColorSensor lsSensor;
	
	public LSLocalizer(Odometer odometer, Navigator navigator, ColorSensor lsSensor) {
		super(odometer, navigator);
		this.lsSensor = lsSensor;
	}
	
	@Override
	public void doLocalization() {
	}
}
