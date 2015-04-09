package main;

import lejos.nxt.LCD;

public class LCDPrinter extends Thread {
	private static final long DISPLAY_PERIOD = 100;
	private final static int LEFT=0, RIGHT=1, FRONT=2;
	private Odometer odometer;
//	private Navigator nav;
	private USController usC;
//	private Object lock;
	
	public LCDPrinter(Odometer odometer, Navigator nav, USController usC, Object lock) {
		this.odometer = odometer;
//		this.nav = nav;
		this.usC = usC;
	}
	
	/**
	 * Starts displaying the odometry values on the LCD of the NXT brick.
	 */
	public void run() {
		long displayStart, displayEnd;
		// clear the display once
		LCD.clearDisplay();

		while (true) {
			displayStart = System.currentTimeMillis();

			// clear the lines for displaying odometry information
			LCD.drawString("X:              ", 0, 0);
			LCD.drawString("Y:              ", 0, 1);
			LCD.drawString("T:              ", 0, 2);
			LCD.drawString("L:              ", 0, 4);
			LCD.drawString("R:              ", 0, 5);
			LCD.drawString("F:              ", 0, 6);

			// display odometry information
			LCD.drawString(formattedDoubleToString(odometer.getX(), 2), 3, 0);
			LCD.drawString(formattedDoubleToString(odometer.getY(), 2), 3, 1);
			LCD.drawString(formattedDoubleToString(odometer.getHeading()*180/Math.PI, 2), 3, 2);
//			LCD.drawString(formattedDoubleToString(usC.getDistance(LEFT), 2), 3, 4);
//			LCD.drawString(formattedDoubleToString(usC.getDistance(RIGHT), 2), 3, 5);
//			LCD.drawString(formattedDoubleToString(usC.getDistance(FRONT), 2), 3, 6);
//			LCD.drawString(formattedDoubleToString(usC.getFilteredDistance(LEFT), 2), 3, 4);
//			LCD.drawString(formattedDoubleToString(usC.getFilteredDistance(RIGHT), 2), 3, 5);
//			LCD.drawString(formattedDoubleToString(usC.getFilteredDistance(FRONT), 2), 3, 6);
//			LCD.drawString(formattedDoubleToString((double)Localizer.distanceFront,2),3,3);
//			LCD.drawString(formattedDoubleToString((double)Localizer.distanceSide,2),3,4);


			// throttle the OdometryDisplay
			displayEnd = System.currentTimeMillis();
			if (displayEnd - displayStart < DISPLAY_PERIOD) {
				try {
					Thread.sleep(DISPLAY_PERIOD - (displayEnd - displayStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that OdometryDisplay will be interrupted
					// by another thread
				}
			}
		}
		}
		
		private static String formattedDoubleToString(double x, int places) {
			String result = "";
			String stack = "";
			long t;
			
			// put in a minus sign as needed
			if (x < 0.0)
				result += "-";
			
			// put in a leading 0
			if (-1.0 < x && x < 1.0)
				result += "0";
			else {
				t = (long)x;
				if (t < 0)
					t = -t;
				
				while (t > 0) {
					stack = Long.toString(t % 10) + stack;
					t /= 10;
				}
				result += stack;
			}
			
			// put the decimal, if needed
			if (places > 0) {
				result += ".";
			
				// put the appropriate number of decimals
				for (int i = 0; i < places; i++) {
					x = Math.abs(x);
					x = x - Math.floor(x);
					x *= 10.0;
					result += Long.toString((long)x);
				}
			}
			
			return result;
		}
}