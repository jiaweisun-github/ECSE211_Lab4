package ca.mcgill.ecse211.lab3;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
//import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
//import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

/**
 * This class is used to define static resources in one place for easy access
 * and to avoid cluttering the rest of the codebase. All resources can be
 * imported at once like this:
 * 
 * <p>
 * {@code import static ca.mcgill.ecse211.lab3.Resources.*;}
 */
public class Resources {

//	public static final double WHEEL_BASE = 2;
	/**
	 * The wheel radius in centimeters.
	 */
	public static final double WHEEL_RAD = 2.130;

	/**
	 * The robot width in centimeters.
	 */
	public static final double TRACK = 15.5;

	/**
	 * The speed at which the robot moves forward in degrees per second.
	 */
	public static final int FORWARD_SPEED = 150;

	/**
	 * The speed at which the robot rotates in degrees per second.
	 */
	public static final int ROTATE_SPEED = 100;

	/**
	 * The motor acceleration in degrees per second squared.
	 */
	public static final int ACCELERATION = 1000;

	/**
	 * Timeout period in milliseconds.
	 */
	public static final int TIMEOUT_PERIOD = 3000;

	/**
	 * The tile size in centimeters.
	 */

	public static final int BAND_CENTER = 30;

	/**
	 * Width of dead band (cm).
	 */
	public static final int BAND_WIDTH = 3;

	/**
	 * Speed of slower rotating wheel (deg/sec).
	 */
	public static final int MOTOR_LOW = (int) (100);

	/**
	 * Speed of the faster rotating wheel (deg/sec).
	 */
	public static final int MOTOR_HIGH = (int) (200);

	public static final double TILE_SIZE = 30.48;
	
	public static final int FILTER_OUT = 20;

	/**
	 * The left motor.
	 */
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	/**
	 * The right motor.
	 */
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	/**
	 * The ultrasonic sensor.
	 */
	public static final EV3UltrasonicSensor US_SENSOR = new EV3UltrasonicSensor(LocalEV3.get().getPort("S4"));
	/**
	 * The LCD.
	 */
	public static final TextLCD LCD = LocalEV3.get().getTextLCD();

	/**
	 * The odometer.
	 */
	public static Odometer odometer = Odometer.getOdometer();

}
