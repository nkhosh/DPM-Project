package tests;
import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import main.Navigator;
import main.RobinHood;


public class odometryCalibration {
	private static  int shootingSpeed = 100;
	private static int shotNumber = 2;
	private static NXTRegulatedMotor launcherMotor = Motor.A;
	private static double radius = 2.15, width = 15.375;
	private static int buttonChoice;
	
	private static RobinHood robinhood;
	
	public static void main(String[] args){
		robinhood = new RobinHood();
		setRadius();
		setWidth();
		LCD.clear();
		LCD.drawString(" Radius="+radius, 0, 2);
		LCD.drawString(" Width="+width, 0, 3);
		
		squareDrive();
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
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
				robinhood.getOdometer().setRadius(radius);
				return;
			}
			buttonChoice = Button.waitForAnyPress();
		}
	}
	public static void setWidth() {
		LCD.clear();
		LCD.drawString("   Set width    ", 0, 0);
		LCD.drawString("       |        ", 0, 1);
		LCD.drawString("< Left | Right >", 0, 2);
		LCD.drawString(" -0.1  |  +0.1  ", 0, 3);
		LCD.drawString("    "+width, 0, 5);
		
		buttonChoice = Button.waitForAnyPress();
		while (buttonChoice != Button.ID_ESCAPE) {
			if( buttonChoice == Button.ID_RIGHT ) {
				width += 0.1;
			}
			else if( buttonChoice == Button.ID_LEFT ) {
				width -= 0.1;
			}
			else if( buttonChoice == Button.ID_ENTER ) {
				break;
			}
			buttonChoice = Button.waitForAnyPress();
		}
		LCD.clear();
		LCD.drawString("   Set width    ", 0, 0);
		LCD.drawString("       |        ", 0, 1);
		LCD.drawString("< Left | Right >", 0, 2);
		LCD.drawString(" -0.01 |  +0.01 ", 0, 3);
		LCD.drawString("    "+width, 0, 5);
		
		buttonChoice = Button.waitForAnyPress();
		
		while( buttonChoice != Button.ID_ESCAPE ) {
			if( buttonChoice == Button.ID_RIGHT ) {
				width += 0.01;
			}
			else if( buttonChoice == Button.ID_LEFT ) {
				width -= 0.01;
			}
			else if( buttonChoice == Button.ID_ENTER ) {
				robinhood.getOdometer().setWidth(width);
				return;
			}
			buttonChoice = Button.waitForAnyPress();
		}
	}
	public static void squareDrive() {
		final double FORWARD_DISTANCE = 60.96;
		final double ROTATION_ANGLE = 90.0;

		// wait 5 seconds
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected that
			// the odometer will be interrupted by another thread
		}

		// drive forward two tiles
		for(int j=0; j<4; j++) {
			robinhood.getNavigator().travelTo(0, FORWARD_DISTANCE);
			robinhood.getNavigator().turnTo(ROTATION_ANGLE, true);
		}
	}
}
