package tests;

import lejos.nxt.Button;
import main.Launcher;
import main.RobinHood;

public class reloaderTest {
	
	static Launcher launcher;
	
	public static void main(String[] args){
		RobinHood robinHood = new RobinHood();
		launcher = robinHood.getLauncher();
		
		int buttonChoice = Button.waitForAnyPress();
		if(buttonChoice == Button.ID_ENTER)
			launcher.launch(6);
		
		buttonChoice = Button.waitForAnyPress();
		while(buttonChoice!=Button.ID_ESCAPE);
		System.exit(0);
		
	}
}
