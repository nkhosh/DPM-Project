package tests;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.TouchSensor;
import main.LCDPrinter;
import main.Odometer;
import main.RobinHood;

public class OdoTesting {
	private static double radius;
	private static double width;
	private static int forwardSpeed;
	private static int rotationSpeed;
	private static double forwardDistance;
	private static double rotationAngle;
	private static int buttonChoice;
	private static Odometer odometer;
	private static LCDPrinter lcdPrinter;
	private static TouchSensor touchSensor = new TouchSensor(SensorPort.S4);

	private final static NXTRegulatedMotor[] wheelMotor = {Motor.A, Motor.B};
	
	public static void main(String[] args){
		RobinHood robinHood = new RobinHood();
		odometer = robinHood.getOdometer();
		lcdPrinter = robinHood.getLcdPrinter();
		
		radius = odometer.getRadius();
		width = odometer.getWidth();
		
		forwardSpeed = 250;
		rotationSpeed = 150;
		forwardDistance = 60.96;
		rotationAngle = 90.0;
		
		do {
			// clear the display
			LCD.clear();

			// ask the user whether the motors should drive in a square or float
			LCD.drawString("< Left | Right >", 0, 0);
			LCD.drawString("       |        ", 0, 1);
			LCD.drawString("  With | Drive  ", 0, 2);
			LCD.drawString("Correc-| in a   ", 0, 3);
			LCD.drawString(" tion  | square ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT
				&& buttonChoice != Button.ID_RIGHT);
		
		robinHood.getOdometer().start();
		robinHood.getLcdPrinter().start();
		
		if (buttonChoice == Button.ID_LEFT) {
			robinHood.getOdometryCorrector().start();
		}
			drive(wheelMotor[0],wheelMotor[1], radius, radius, width );
			buttonChoice = Button.waitForAnyPress();
			while(buttonChoice != Button.ID_ESCAPE){
				if(buttonChoice==Button.ID_LEFT){
					odometer.setX(0);
					odometer.setY(0);
					odometer.setHeading(0);
					drive(wheelMotor[0],wheelMotor[1], radius, radius, width );
				}
				buttonChoice = Button.waitForAnyPress();
			}
		
		System.exit(0);
	}

	public static void drive(NXTRegulatedMotor leftMotor, NXTRegulatedMotor rightMotor,
			double leftRadius, double rightRadius, double width) {
		// reset the motors
		for (NXTRegulatedMotor motor : new NXTRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}

		// wait 5 seconds
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected that
			// the odometer will be interrupted by another thread
		}

		for (int i = 0; i < 4; i++) {
			// drive forward two tiles
			leftMotor.setSpeed(forwardSpeed);
			rightMotor.setSpeed(forwardSpeed);

			leftMotor.rotate(convertDistance(leftRadius, forwardDistance), true);
			rightMotor.rotate(convertDistance(rightRadius, forwardDistance), false);

			// turn 90 degrees clockwise
			leftMotor.setSpeed(rotationSpeed);
			rightMotor.setSpeed(rotationSpeed);

			leftMotor.rotate(convertAngle(leftRadius, width, rotationAngle), true);
			rightMotor.rotate(-convertAngle(rightRadius, width, rotationAngle), false);
		}
		
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
