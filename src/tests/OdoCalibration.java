package tests;
import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.util.Delay;
import main.Navigator;
import main.RobinHood;

/**
 * Test class to determine the constants defined in the odometer, like radius of the wheels and the distance of the wheels.
 * This class was subject to change and addition and deletion of many lines of code for different tests.
 * For the different versions of the test class please refer to Git.
 * @author Niloofar Khoshsiyar
 *
 */
public class OdoCalibration {
	
	private static double radius = 2.055, wheelsDistance = 15.85; // 16.37    2.09, wheelsDistance=16.35
	private static int buttonChoice;
	private final static NXTRegulatedMotor[] wheelMotor = {Motor.A, Motor.B};
	private static Navigator nav;
	
	private static RobinHood robinhood;
	
	public static void main(String[] args){
		robinhood = new RobinHood();
		nav = robinhood.getNavigator();
		while ( true ) {
			LCD.clear();
			LCD.drawString("< Left | Right >", 0, 2);
			LCD.drawString("radius | width  ", 0, 3);
			buttonChoice = Button.waitForAnyPress();
			if( buttonChoice == Button.ID_RIGHT ) {
				setWidth();
			}
			else if( buttonChoice == Button.ID_LEFT ) {
				setRadius();
			}
			else if( buttonChoice ==Button.ID_ESCAPE )
				break;
			LCD.clear();
			LCD.drawString(" Radius="+radius, 0, 2);
			LCD.drawString(" Width="+wheelsDistance, 0, 3);
		}
		System.exit(0);
	}
	
	public static void setRadius() {
		LCD.clear();
		LCD.drawString("   Set radius   ", 0, 0);
		LCD.drawString("       |        ", 0, 1);
		LCD.drawString("< Left | Right >", 0, 2);
		LCD.drawString(" -0.1  |  +0.1  ", 0, 3);
		LCD.drawString("    "+radius, 0, 5);
		
		buttonChoice = Button.waitForAnyPress();
		while (buttonChoice != Button.ID_ESCAPE) {
			if( buttonChoice == Button.ID_RIGHT ) {
				radius += 0.1;
			}
			else if( buttonChoice == Button.ID_LEFT ) {
				radius -= 0.1;
			}
			else if( buttonChoice == Button.ID_ENTER ) {
				break;
			}
			LCD.drawString("    "+radius, 0, 5);
			buttonChoice = Button.waitForAnyPress();
		}
		LCD.clear();
		LCD.drawString("   Set radius   ", 0, 0);
		LCD.drawString("       |        ", 0, 1);
		LCD.drawString("< Left | Right >", 0, 2);
		LCD.drawString(" -0.01 |  +0.01 ", 0, 3);
		LCD.drawString("    "+radius, 0, 5);
		
		buttonChoice = Button.waitForAnyPress();
		
		while( buttonChoice != Button.ID_ESCAPE ) {
			if( buttonChoice == Button.ID_RIGHT ) {
				radius += 0.01;
			}
			else if( buttonChoice == Button.ID_LEFT ) {
				radius -= 0.01;
			}
			else if( buttonChoice == Button.ID_ENTER ) {
				robinhood.getOdometer().start();
				robinhood.getOdometer().setRadius(radius);
				goForward();
//				squareDrive();
				return;
			}
			LCD.drawString("    "+radius, 0, 5);
			buttonChoice = Button.waitForAnyPress();
		}
	}
	public static void setWidth() {
		LCD.clear();
		LCD.drawString("   Set width    ", 0, 0);
		LCD.drawString("       |        ", 0, 1);
		LCD.drawString("< Left | Right >", 0, 2);
		LCD.drawString(" -0.1  |  +0.1  ", 0, 3);
		LCD.drawString("    "+wheelsDistance, 0, 5);
		
		buttonChoice = Button.waitForAnyPress();
		while (buttonChoice != Button.ID_ESCAPE) {
			if( buttonChoice == Button.ID_RIGHT ) {
				wheelsDistance += 0.1;
			}
			else if( buttonChoice == Button.ID_LEFT ) {
				wheelsDistance -= 0.1;
			}
			else if( buttonChoice == Button.ID_ENTER ) {
				break;
			}
			LCD.drawString("    "+wheelsDistance, 0, 5);
			buttonChoice = Button.waitForAnyPress();
		}
		LCD.clear();
		LCD.drawString("   Set width    ", 0, 0);
		LCD.drawString("       |        ", 0, 1);
		LCD.drawString("< Left | Right >", 0, 2);
		LCD.drawString(" -0.01 |  +0.01 ", 0, 3);
		LCD.drawString("    "+wheelsDistance, 0, 5);
		
		buttonChoice = Button.waitForAnyPress();
		
		while( buttonChoice != Button.ID_ESCAPE ) {
			if( buttonChoice == Button.ID_RIGHT ) {
				wheelsDistance += 0.01;
			}
			else if( buttonChoice == Button.ID_LEFT ) {
				wheelsDistance -= 0.01;
			}
			else if( buttonChoice == Button.ID_ENTER ) {
				Delay.msDelay(500);
				robinhood.getOdometer().start();
				robinhood.getOdometer().setWidth(wheelsDistance);
				Delay.msDelay(500);
				turnAngle();
//				squareDrive();
				return;
			}
			LCD.drawString("    "+wheelsDistance, 0, 5);
			buttonChoice = Button.waitForAnyPress();
		}
	}
	
	public static void squareDrive() {
//		final double FORWARD_DISTANCE = 60.96;
		final double FORWARD_DISTANCE = 6*30.48;
		final double ROTATION_ANGLE = 90.0;
		wheelMotor[0].setSpeed(200);
		wheelMotor[1].setSpeed(200);
		// wait 5 seconds
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected that
			// the odometer will be interrupted by another thread
		}

		// drive forward two tiles
		for(int j=0; j<4; j++) {
			wheelMotor[0].rotate((int)Navigator.convertDistanceToAngle(radius, FORWARD_DISTANCE), true);
			wheelMotor[1].rotate((int)Navigator.convertDistanceToAngle(radius, FORWARD_DISTANCE), false);

			wheelMotor[0].rotate((int)Navigator.convertHeadingToWheelAngle(radius, wheelsDistance, ROTATION_ANGLE), true);
			wheelMotor[1].rotate(-(int)Navigator.convertHeadingToWheelAngle(radius, wheelsDistance, ROTATION_ANGLE), false);
		}
	}
	public static void goForward() {
		final double FORWARD_DISTANCE = (9*30.48);
		nav.travelTo(0, FORWARD_DISTANCE, false);
	}
	public static void turnAngle() {
		final double ROTATION_ANGLE = 360.0*3;
		wheelMotor[0].setSpeed(123);
		wheelMotor[1].setSpeed(123);

		wheelMotor[0].rotate((int)Navigator.convertHeadingToWheelAngle(radius, wheelsDistance, ROTATION_ANGLE), true);
		wheelMotor[1].rotate(-(int)Navigator.convertHeadingToWheelAngle(radius, wheelsDistance, ROTATION_ANGLE), false);
	}
}
