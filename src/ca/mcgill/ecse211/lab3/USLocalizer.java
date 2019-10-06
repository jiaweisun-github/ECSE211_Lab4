package ca.mcgill.ecse211.lab3;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class USLocalizer extends UltrasonicController{

	private Odometer odo;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private UltrasonicController controller;
//	private EV3UltrasonicSensor USSensor;
//	private UltrasonicPoller poller;
	private SampleProvider USDistance;
//	private static float distance;
	public static boolean isRisingEdge;
	final TextLCD display = LocalEV3.get().getTextLCD();
	
	public int filterControl = 0;
	private static double pi = Math.PI;
	private int distance;
	
	UltrasonicPoller poller = new UltrasonicPoller(USDistance, controller);
	
	public USLocalizer(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, boolean isRisingEdge, SampleProvider usDistance){		
		this.odo = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		USLocalizer.isRisingEdge = isRisingEdge;
		this.USDistance = usDistance;
		Navigation navigation = new Navigation(odometer, leftMotor, rightMotor);
		
	}
	public void edge() {
		if(!isRisingEdge) {
			risingEdge();
		}
		else {
			fallingEdge();
		}
		
	}
	
	private void risingEdge() {
		
		distance = fetchUSData();
		int counter = 0;
		double angle1, angle2, rotationTheta;
		
		
		leftMotor.setSpeed(Resources.ROTATE_SPEED);
		rightMotor.setSpeed(Resources.ROTATE_SPEED);
		
		if(distance > Resources.TILE_SIZE) {
			while(distance > Resources.TILE_SIZE) {						//turn right until hits wall
				leftMotor.forward();
				rightMotor.backward();
			}
			
			angle1 = odo.getXYT()[2];
//		yPosition = Resources.TILE_SIZE*Math.sin(angle1-pi/2);
			
			while(distance >= Resources.TILE_SIZE) {					//turn left until hits wall
				leftMotor.backward();
				rightMotor.forward();
			}
			
			angle2 = odo.getXYT()[2];
			
			rotationTheta = -(((angle1-angle2)/2)-angle1);
			
//		xPosition = Resources.TILE_SIZE
			
		}
		else {
			if(counter == 0) {
				while(distance<Resources.TILE_SIZE) {						//turn right until sensor detects TILE_SIZE distance
					leftMotor.forward();
					rightMotor.backward();
				}
				counter++;													//counter = 1
			}
			else if(counter == 1) {
				while(distance>Resources.TILE_SIZE) {						//turn right still, until sensor detects second TILE_SIZE distance
					leftMotor.forward();
					rightMotor.backward();
				}
			}
			angle1 = odo.getXYT()[2];										//angle 1 = odometer reading
			
			while(distance>=Resources.TILE_SIZE) {							//turn left until sensor detects TILE_SIZE distance
				leftMotor.backward();
				rightMotor.forward();
			}
				
			angle2 = odo.getXYT()[2];										//angle 2 = odometer reading
			
			rotationTheta = ((angle1-angle2)/2)+angle2;						//calculate angle by which the robot should turn to reorient
		}
		leftMotor.rotate(thetaToDegree(rotationTheta));						//rotate to the original axis
		rightMotor.rotate(-thetaToDegree(rotationTheta));
		
	}
	
	private void fallingEdge() {
		
	}
	@Override
	public void processUSData (int distance) {
		// rudimentary filter - toss out invalid samples corresponding to null signal 
	    if (distance >= 255 && filterControl < Resources.FILTER_OUT) {
	      // bad value: do not set the distance var, do increment the filter value
	      this.filterControl++;
	    } else if (distance >= 255) {
	      // We have repeated large values, so there must actually be nothing
	      // there: leave the distance alone
	      this.distance = distance;
	    } else {
	      // distance went below 255: reset filter and leave
	      // distance alone.
	      this.filterControl = 0;
	      this.distance = distance;
	    }
	}
	
	@Override
	public int readUSDistance() {
		return this.distance;
	}
	
	public int fetchUSData() {
		USDistance.fetchSample(poller.usData, 0);;
		return (int) (poller.usData[0] * 100);
		
	}
	
	public int thetaToDegree(double rotationTheta) {
		rotationTheta = rotationTheta*180/pi;
		return (int) rotationTheta;
	}
}


