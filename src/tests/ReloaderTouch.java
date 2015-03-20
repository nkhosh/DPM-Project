package tests;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.SensorPort;
import lejos.nxt.TouchSensor;
import main.Launcher;
import main.RobinHood;

public class ReloaderTouch {
	static Launcher launcher;
	
	public static void main(String[] args){
		RobinHood robinHood = new RobinHood();
		launcher = robinHood.getLauncher();
		TouchSensor touchSensor = new TouchSensor(SensorPort.S4);
		
		LCD.drawString("Orange button to fire!", 0, 0);
		
		while(true){
			while(!touchSensor.isPressed());
			launcher.launch(6);
		}
		
	}
}
