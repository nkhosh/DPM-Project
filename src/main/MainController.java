package main;

import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;

public class MainController {
	private final static NXTRegulatedMotor[] wheelMotor = {Motor.A, Motor.B};

	public static void main(String[] args) {
		RobinHood robinHood = new RobinHood();
	}
}
