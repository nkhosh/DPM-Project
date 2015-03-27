package main;


import lejos.nxt.LCD;
import lejos.util.Timer;
import lejos.util.TimerListener;

public class LCDInfo implements TimerListener{
	public static final int LCD_REFRESH = 100;
	private Odometer odo;
	private Timer lcdTimer;
	
	// arrays for displaying data
	private double [] pos;
	
	public LCDInfo(Odometer odo) {
		this.odo = odo;
		this.lcdTimer = new Timer(LCD_REFRESH, this);
		
		// initialise the arrays for displaying data
		pos = new double [3];
		
		// start the timer
		lcdTimer.start();
	}
	
	public void timedOut() { 

		odo.getPosition(pos);
		LCD.clear();
		LCD.drawString("X: ", 0, 0);
		LCD.drawString("Y: ", 0, 1);
		LCD.drawString("H: ", 0, 2);
		LCD.drawInt((int)((pos[0] * 1000)/3048), 3, 0);
		LCD.drawInt((int)((pos[1] * 1000)/3048), 3, 1);
		LCD.drawInt((int)pos[2], 3, 2);
		LCD.drawInt((int)Localizer.l, 3, 4);
		LCD.drawInt((int)Localizer.r, 3, 5);
		LCD.drawInt((int)Localizer.gridCounter, 3, 6);

		
	}
}



