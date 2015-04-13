package main;
import lejos.nxt.NXTRegulatedMotor;

/**
 * This class is responsible for launching and reloading the ping pong balls.
 * It keeps track of the number of the balls inside the holder and determines the number of reloadings accordingly.
 * @author Niloofar Khoshsiyar
 *
 */
public class Launcher {
	// The array of the motor speed of the reloader rod for ping pong balls in the quiver from number 5 to 1
	private static final int[] RELOAD_SPEEDS = {100,375,367,374,380};
	private static final int SHOOTING_SPEED = 200;
	
	private NXTRegulatedMotor launcherMotor;
	private int ballsInQuiver;
	
	public Launcher(NXTRegulatedMotor launcherMotor,int startingBalls) {
		this.launcherMotor = launcherMotor;
		this.ballsInQuiver = startingBalls;
	}
	
	/**
	 * Launches the ping pong balls
	 * @param balls The number of balls to be shot
	 */
	public void launch(int balls){
		// fire specified number of balls, reloading in between each shot
		for(int i=0; i<balls; i++){
			launcherMotor.setSpeed(SHOOTING_SPEED);
			launcherMotor.rotate(-180);
			reload();
		}
	}
	
	/**
	 * Reloads a ball to the launching platform
	 */
	private void reload(){
		if(ballsInQuiver!=0){
			launcherMotor.setSpeed(RELOAD_SPEEDS[ballsInQuiver - 1]);
			launcherMotor.rotate(-180);
			ballsInQuiver--;
		}
		else{
			launcherMotor.rotate(-180);
			ballsInQuiver = 5;
		}
	}
	
	/**
	 * Sets the number of ping pong balls inside the quiver
	 * @param balls
	 */
	public void setBallsInQuiver(int balls){
		this.ballsInQuiver = balls;
	}
	
	
}