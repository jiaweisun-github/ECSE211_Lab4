package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
//import ca.mcgill.ecse211.lab3.Resources.*;


public class Navigation extends Thread {

		private Odometer odo;
		private EV3LargeRegulatedMotor leftMotor, rightMotor;
		private static final double pi = Math.PI;
		private static boolean navigating = false;

		//distance and theta calculation parameters
		double xDistance;
		double yDistance;
		double distance;
		double angle = 0;
		double angleRad;
		double positiveTheta;
		/**
		 * Constructor
		 * 
		 * @param odo
		 * @param leftMotor
		 * @param rightMotor
		 */
		public Navigation(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
			this.odo = odo;
			this.leftMotor = leftMotor;
			this.rightMotor = rightMotor;
		}

		@Override
		public void run() {
			
			double TILE_SIZE = Resources.TILE_SIZE;
			// Input travel points here
			travelTo((2 * TILE_SIZE), (1 * TILE_SIZE));
			travelTo((0 * TILE_SIZE), (2 * TILE_SIZE));
			travelTo((0 * TILE_SIZE), (1 * TILE_SIZE));
			travelTo((2 * TILE_SIZE), (2 * TILE_SIZE));
			travelTo((1 * TILE_SIZE), (2 * TILE_SIZE));
		}

		public void travelTo(double x, double y) {

			leftMotor.setSpeed(0);;
			rightMotor.setSpeed(0);
			leftMotor.setAcceleration(3000);
			rightMotor.setAcceleration(3000);

			navigating = true;

			// Calculate path and angle
			xDistance = x - odo.getXYT()[0];
			yDistance = y - odo.getXYT()[1];
			distance = Math.sqrt(Math.pow(xDistance, 2)+Math.pow(yDistance, 2));
			
			if(xDistance > 0 && yDistance > 0) {
//				if(odo.getXYT()[2]<0) {
//					positiveTheta = odo.getXYT()[2] + 2*pi;
//				}
				angle = pi/2 - positiveTheta - Math.atan(yDistance/xDistance); //DONEEEEEE							//positive x and y, 1ST QUADRANT GOOD
				
			}
			else if(xDistance < 0 && yDistance > 0) {													//negative x and positive y, 2ND QUADRANT GOOD
//				if(odo.getXYT()[2]<0) {
//				if((Math.toRadians(odo.getXYT()[2]) >= 3*pi/2 && Math.toRadians(odo.getXYT()[2]) < 2*pi)|| 								//DONNEEEEE
//						(Math.toRadians(odo.getXYT()[2]) <= 0 && Math.toRadians(odo.getXYT()[2]) > -pi/2)) {
//					if(Math.toRadians(odo.getXYT()[2])>3*pi/2+Math.atan(yDistance/xDistance)) {					//case 1
//						angle = 3*pi/2 - Math.atan(yDistance/xDistance) - Math.toRadians(odo.getXYT()[2]);
//					}
//					else {																						//case 2
//						angle = 3*pi/2 + Math.atan(yDistance/xDistance) - Math.toRadians(odo.getXYT()[2]);
//					}
//				}
//				else if(Math.toRadians(odo.getXYT()[2]) >= 0 && Math.toRadians(odo.getXYT()[2])< pi/2 ||
//						(Math.toRadians(odo.getXYT()[2]) <= -3*pi/2 && Math.toRadians(odo.getXYT()[2]) > -2*pi)) {
//					
//				}
////				
//				else {
//					angle = 3*pi/2 - Math.toRadians(odo.getXYT()[2] + Math.atan(yDistance/xDistance));
//				}
//				if(odo.getXYT()[2]<0) {
//				positiveTheta = odo.getXYT()[2] + 2*pi;
//			}
				angle = 3*pi/2 - positiveTheta - Math.atan(yDistance/xDistance);

			}
			else if(xDistance < 0 && yDistance < 0) {													//negative x and negative y, 3RD QUADRANT GOOD
//				if(odo.getXYT()[2]<0) {
//					positiveTheta = odo.getXYT()[2] + 2*pi;
//				}
				angle = 3*pi/2 - positiveTheta - Math.atan(yDistance/xDistance);
			}
			else if(xDistance > 0 && yDistance < 0) {													//positive x and negative y, 4TH QUADRANT GOOD
//				if(odo.getXYT()[2]<0) {
//					positiveTheta = odo.getXYT()[2] + 2*pi;
//				}
				angle = pi/2 - positiveTheta - Math.atan(yDistance/xDistance);
			}
			System.out.println("           "+angle);
			
			// Turn to face the waypoint
			int ROTATE_SPEED = Resources.ROTATE_SPEED;
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			turnBy(angle);

			// Advance forward equal to path distance
			int FORWARD_SPEED = Resources.FORWARD_SPEED;

			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			leftMotor.rotate(distanceToRotations(distance), true);
			rightMotor.rotate(distanceToRotations(distance), false);
		}

		public void turnBy(double theta) {
			double myAngle = theta;
			if (myAngle > pi) {
				myAngle = -(2*pi - angle);
			} 
			else if(myAngle > 2*pi) {
				myAngle = -(3*pi - angle);
			}
			else if(myAngle > 3*pi) {
				myAngle = -(4*pi - angle);
			}
			else if(myAngle > 4*pi) {
				myAngle = -(5*pi - angle);
			}
			else if (myAngle < -pi) {
				myAngle = 2 * pi + angle;
			}
			else if (myAngle < -2*pi) {
				myAngle = 3 * pi + angle;
			}
			else if (myAngle < -3*pi) {
				myAngle = 4 * pi + angle;
			}
			else if (myAngle < -4*pi) {
				myAngle = 5 * pi + angle;
			}
			//			if (theta - odo.getXYT()[2] > pi/2) {											//if the angle theta at which it turns is bigger than pi, new angle = angle - 2*pi
			//				myAngle = theta - 2 * pi;
			//			} else if (theta - odo.getXYT()[2] < -pi) {
			//				myAngle = 2 * pi + theta;
			//			}

			leftMotor.rotate(radianToDegree(myAngle), true);
			rightMotor.rotate(-(radianToDegree(myAngle)), false);
		}
//		public void turnTo(double theta) {
//			double angle = getMinAngle(theta - odo.getTheta());
//
//			//double angle = theta - odometer.getTheta();
//
//			leftMotor.rotate(radianToDegree(angle), true);
//			rightMotor.rotate(-radianToDegree(angle), false);
//		}
//
//		public double getMinAngle(double angle) {
//			if (angle > pi) {
//				angle = angle - 2 * pi;
//			} else if (angle < -pi) {
//				angle = 2 * pi + angle;
//			}
//			return angle;
//		}

		public int distanceToRotations(double distance) {
			return (int) (180 * distance / (pi * Resources.WHEEL_RAD));
		}

		public int radianToDegree(double angle) {
			return distanceToRotations(Resources.TRACK * angle / 2);
		}
		
		/**
		 * Checks to see if the robot is on the move or not
		 * 
		 * @return true/false
		 */
		public boolean isNavigating() {
			return navigating;
		}

	}

