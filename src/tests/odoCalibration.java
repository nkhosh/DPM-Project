package tests;
import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import main.Navigator;
import main.RobinHood;

public class odoCalibration {
	private static double radius = 2.075, width = 16.22;
	private static int buttonChoice;
	private final static NXTRegulatedMotor[] wheelMotor = {Motor.A, Motor.B};
	private double[] tachometer;
	
	private static RobinHood robinhood;
	
	public static void main(String[] args){
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
			LCD.drawString(" Width="+width, 0, 3);
//			if(buttonChoice==Button.ID_ENTER)
//				squareDrive();
//			if( Button.waitForAnyPress()==Button.ID_ESCAPE)
//				break;
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
				robinhood.getOdometer().setRadius(radius);
				goForward();
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
			LCD.drawString("    "+width, 0, 5);
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
				turnAngle();
				return;
			}
			LCD.drawString("    "+width, 0, 5);
			buttonChoice = Button.waitForAnyPress();
		}
	}
	
	public static void squareDrive() {
		final double FORWARD_DISTANCE = 60.96;
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
//			robinhood.getNavigator().travelTo((j%2)*FORWARD_DISTANCE, ((j+1)%2)*FORWARD_DISTANCE);
//			robinhood.getNavigator().turnTo(ROTATION_ANGLE*(j+1), true);
			wheelMotor[0].rotate((int)Navigator.convertDistance(radius, FORWARD_DISTANCE), true);
			wheelMotor[1].rotate((int)Navigator.convertDistance(radius, FORWARD_DISTANCE), false);

			wheelMotor[0].rotate((int)Navigator.convertAngle(radius, width, ROTATION_ANGLE), true);
			wheelMotor[1].rotate(-(int)Navigator.convertAngle(radius, width, ROTATION_ANGLE), false);
		}
	}
	public static void goForward() {
		final double FORWARD_DISTANCE = 91.44;

		// wait 5 seconds
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected that
			// the odometer will be interrupted by another thread
		}

//		robinhood.getNavigator().travelTo(0, FORWARD_DISTANCE);
		wheelMotor[0].rotate((int)Navigator.convertDistance(radius, FORWARD_DISTANCE), true);
		wheelMotor[1].rotate((int)Navigator.convertDistance(radius, FORWARD_DISTANCE), false);
	}
	public static void turnAngle() {
		final double ROTATION_ANGLE = 360.0;

		// wait 5 seconds
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected that
			// the odometer will be interrupted by another thread
		}

//		robinhood.getNavigator().turnTo(ROTATION_ANGLE, true);

		wheelMotor[0].rotate((int)Navigator.convertAngle(radius, width, ROTATION_ANGLE), true);
		wheelMotor[1].rotate(-(int)Navigator.convertAngle(radius, width, ROTATION_ANGLE), false);
	}
}
