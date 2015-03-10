package main;

public abstract class Localizer {
	private Odometer odometer;
	private Navigator navigator;
	
	public Localizer(Odometer odometer, Navigator navigator) {
		this.odometer = odometer;
		this.navigator = navigator;
	}
	
	public void doLocalization() {
		
	}
}
