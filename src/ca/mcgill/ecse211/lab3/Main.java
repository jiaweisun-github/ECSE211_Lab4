package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.lab3.UltrasonicPoller;
import lejos.hardware.Button;
//import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * The main driver class for the odometry lab.
 */
public class Main {

	/**
	 * The main entry point.
	 * 
	 * @param args
	 */
	public static boolean isRising;
	public static UltrasonicController controller;
	public static USLocalizer localizer;
	public static void main(String[] args) {

		int buttonChoice;
		buttonChoice = chooseNavigationMode();
		
		SampleProvider usDistance = Resources.US_SENSOR.getMode("Distance");
		
		new Thread(Resources.odometer).start();

		if (buttonChoice == Button.ID_LEFT) {
			isRising = true;
			localizer = new USLocalizer(Resources.odometer, Resources.leftMotor, Resources.rightMotor, isRising, usDistance);
			localizer.edge();
		}       
		else if (buttonChoice == Button.ID_RIGHT) {
			isRising = false;
			localizer = new USLocalizer(Resources.odometer, Resources.leftMotor, Resources.rightMotor, isRising, usDistance);
			localizer.edge();
		}
		new Thread(new UltrasonicPoller(usDistance, controller)).start();
		new Thread(new Display()).start();
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
		} // do nothing

		System.exit(0);
	}

	/**
	 * Asks the user whether cthey want to choose navigation or navigation with obstacles.
	 * 
	 * @return the user choice
	 */
	private static int chooseNavigationMode() {
		int buttonChoice;
		Display.showText("< Left | Right >",
						 "       |        ", 
						 "Rising |Falling ",
						 "edge   |edge    ",
						 "       |		  ");

		do {
			buttonChoice = Button.waitForAnyPress(); // left or right press
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
		return buttonChoice;
	}

	/**
	 * Asks the user whether odometry correction should be run or not.
	 * 
	 * @return the user choice
	 */


	/**
	 * Sleeps current thread for the specified duration.
	 * 
	 * @param duration sleep duration in milliseconds
	 */
	public static void sleepFor(long duration) {
		try {
			Thread.sleep(duration);
		} catch (InterruptedException e) {
			// There is nothing to be done here
		}
	}

}
