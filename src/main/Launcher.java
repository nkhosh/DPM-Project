package main;
import lejos.nxt.NXTRegulatedMotor;

public class Launcher {
	private static final int[] reloadSpeeds = {235,375,367,374,380};
	
	private static final int shootingSpeed = 200;
	private NXTRegulatedMotor launcherMotor;
	private int ballsInQuiver;
	
	public Launcher(NXTRegulatedMotor launcherMotor,int startingBalls) {
		this.launcherMotor = launcherMotor;
		ballsInQuiver = startingBalls;
	}
	
	/**
	 * Launches the ping pong balls
	 * @param balls The number of balls to be shot
	 */
	public void launch(int balls){
		
		// fire specified number of balls, reloading in between each shot
		for(int i=0; i<balls; i++){
			launcherMotor.setSpeed(shootingSpeed);
			launcherMotor.rotate(-180);
			if(ballsInQuiver!=0)
				reload();
		}
		
		// return to initial position
		launcherMotor.rotate(-180);
		
		// reset balls in quiver (for testing)
		ballsInQuiver = 5;
	}
	
	private void reload(){
		launcherMotor.setSpeed(reloadSpeeds[ballsInQuiver - 1]);
		launcherMotor.rotate(-180);
		ballsInQuiver--;
	}
	
	
}