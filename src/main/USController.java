package main;
import lejos.nxt.UltrasonicSensor;


public class USController {
	UltrasonicSensor[] us;
	public USController(UltrasonicSensor[] usSensor) {
		us = usSensor;
	}
}
