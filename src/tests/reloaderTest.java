package tests;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import main.Launcher;
import main.RobinHood;

public class ReloaderTest {
	
	static Launcher launcher;
	
	public static void main(String[] args){
		RobinHood robinHood = new RobinHood();
		launcher = robinHood.getLauncher();
		
		       
		int buttonChoice = Button.waitForAnyPress();
		
		LCD.drawString("Orange button to fire!", 0, 0);
		
		while(buttonChoice!=Button.ID_ESCAPE){
			if(buttonChoice == Button.ID_ENTER){
				launcher.launch(6);
			}
				
			buttonChoice = Button.waitForAnyPress();
		}
		
		buttonChoice = Button.waitForAnyPress();
		while(buttonChoice!=Button.ID_ESCAPE);
		System.exit(0);
		
	}
}
