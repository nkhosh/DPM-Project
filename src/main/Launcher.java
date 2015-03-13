package main;
import lejos.nxt.NXTRegulatedMotor;

public class Launcher {
	private static final int[] reloadSpeeds = {315,355,365,375,395};
	private static final int shootingSpeed = 200;
	private Odometer odometer;
	private NXTRegulatedMotor launcherMotor;
	private int ballsInQuiver;
	
	public Launcher(Odometer odometer, NXTRegulatedMotor launcherMotor) {
		this.odometer = odometer;
		this.launcherMotor = launcherMotor;
		ballsInQuiver = 5;
	}
	
	public void launch(int balls){
		for(int i=0; i<balls; i++){
			launcherMotor.setSpeed(shootingSpeed);
			launcherMotor.rotate(-180);
			if(ballsInQuiver!=0)
				reload();
		}
	}
	
	private void reload(){
		launcherMotor.setSpeed(reloadSpeeds[ballsInQuiver - 1]);
		launcherMotor.rotate(-180);
		ballsInQuiver--;
	}
}
