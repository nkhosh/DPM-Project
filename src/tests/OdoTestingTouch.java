package tests;

import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.TouchSensor;
import main.Odometer;
import main.RobinHood;

/**
 * Testing of the odometer by using touch sensor for ease of the use.
 * This class was subject to change and addition and deletion of many lines of code for different tests.
 * For the different versions of the test class please refer to Git.
 * @author Nil
 *
 */
public class OdoTestingTouch {
	private static double radius;
	private static double width;
	private static int forwardSpeed;
	private static int rotationSpeed;
	private static double forwardDistance;
	private static double rotationAngle;
	private static Odometer odometer;
	private static TouchSensor touchSensor;

	private final static NXTRegulatedMotor[] wheelMotor = { Motor.A, Motor.B };

	public static void main(String[] args) {
		RobinHood robinHood = new RobinHood();
		touchSensor = new TouchSensor(SensorPort.S4);
		odometer = robinHood.getOdometer();

		radius = odometer.getRadius();
		width = odometer.getWheelsDistance();

		forwardSpeed = 250;
		rotationSpeed = 150;
		forwardDistance = 60.96;
		rotationAngle = 90.0;

		// do {
		// clear the display
		// LCD.clear();

		// ask the user whether the motors should drive in a square or float
		LCD.drawString("< Left | Right >", 0, 0);
		LCD.drawString("       |        ", 0, 1);
		LCD.drawString("  With | Drive  ", 0, 2);
		LCD.drawString("Correc-| in a   ", 0, 3);
		LCD.drawString(" tion  | square ", 0, 4);

		// buttonChoice = Button.waitForAnyPress();
		// }
		while (!touchSensor.isPressed());

		LCD.clear();
		robinHood.getOdometer().start();
		robinHood.getLCDPrinter().start();

		// if (buttonChoice == Button.ID_LEFT) {
		robinHood.getOdometryCorrector().start();
		// }
		drive(wheelMotor[0], wheelMotor[1], radius, radius, width);
		// buttonChoice = Button.waitForAnyPress();
		// while(buttonChoice != Button.ID_ESCAPE){
		// if(buttonChoice==Button.ID_LEFT){
		while (true) {
			while (!touchSensor.isPressed());
			odometer.setX(0);
			odometer.setY(0);
			odometer.setHeading(0);
			drive(wheelMotor[0], wheelMotor[1], radius, radius, width);
		}
		// }
		// buttonChoice = Button.waitForAnyPress();
		// }

	}

	public static void drive(NXTRegulatedMotor leftMotor,
			NXTRegulatedMotor rightMotor, double leftRadius,
			double rightRadius, double width) {
		// reset the motors
		for (NXTRegulatedMotor motor : new NXTRegulatedMotor[] { leftMotor,
				rightMotor }) {
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

			leftMotor
					.rotate(convertDistance(leftRadius, forwardDistance), true);
			rightMotor.rotate(convertDistance(rightRadius, forwardDistance),
					false);

			// turn 90 degrees clockwise
			leftMotor.setSpeed(rotationSpeed);
			rightMotor.setSpeed(rotationSpeed);

			leftMotor.rotate(convertAngle(leftRadius, width, rotationAngle),
					true);
			rightMotor.rotate(-convertAngle(rightRadius, width, rotationAngle),
					false);
		}

	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
