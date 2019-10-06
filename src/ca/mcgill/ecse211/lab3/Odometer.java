package ca.mcgill.ecse211.lab3;

//import static ca.mcgill.ecse211.lab2.Resources.colorSensor;
//import static ca.mcgill.ecse211.lab2.Resources.odometer;
import static ca.mcgill.ecse211.lab3.Resources.*;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import lejos.robotics.SampleProvider;

/**
 * The odometer class keeps track of the robot's (x, y, theta) position.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 * @author Younes Boubekeur
 */

public class Odometer implements Runnable{
//	public float[] dataArray;
//	private SampleProvider colorDetector;
  /**
   * The x-axis position in cm.
   */
  private volatile double x;
  
  /**
   * The y-axis position in cm.
   */
  private volatile double y; // y-axis position
  
  /**
   * The orientation in degrees.
   */
  private volatile double theta; // Head angle
  
  /**
   * The (x, y, theta) position as an array
   */
  private double[] position;

  // Thread control tools
  /**
   * Fair lock for concurrent writing
   */
  private static Lock lock = new ReentrantLock(true);
  
  /**
   * Indicates if a thread is trying to reset any position parameters
   */
  private volatile boolean isResetting = false;

  /**
   * Lets other threads know that a reset operation is over.
   */
  private Condition doneResetting = lock.newCondition();
    
  private static Odometer odo;

  // Motor-related variables
  private static int lastLeftMotorTachoCount;								//last tachometer count, left and right wheel
  private static int lastRightMotorTachoCount;
  private static int currentLeftMotorTachoCount;							//current tachometer count, left and right
  private static int currentRightMotorTachoCount;
  

  /**
   * The odometer update period in ms.
   */
  private static final long ODOMETER_PERIOD = 25;

  
  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   */
  public Odometer() {
//	  this.odo = odometer;
//	  this.dataArray = new float[colorSensor.sampleSize()];
	  setXYT(0, 0, 0);
  }
  														
  /**
   * Returns the Odometer Object. Use this method to obtain an instance of Odometer.
   * 
   * @return the Odometer Object
   */
  public synchronized static Odometer getOdometer() {
    if (odo == null) {
      odo = new Odometer();
    }
    
    return odo;
  }

  /**
   * This method is where the logic for the odometer will run.
   */
  public void run() {
    long updateStart, updateEnd;
    // Clear tacho counts and put motors in freewheel mode. Then initialize tacho count 
    // variable to its current state.
    leftMotor.resetTachoCount();
    rightMotor.resetTachoCount(); 
    lastLeftMotorTachoCount=leftMotor.getTachoCount(); 
    lastRightMotorTachoCount=rightMotor.getTachoCount();
    
    while (true) {
    	double distL, distR, deltaD, deltaT, dX, dY;
    	updateStart = System.currentTimeMillis();
    	
//    	theta = odo.getXYT()[2]*180/Math.PI;	
    	currentLeftMotorTachoCount = leftMotor.getTachoCount();
    	currentRightMotorTachoCount = rightMotor.getTachoCount();
    	distL = 3.14159*WHEEL_RAD*(currentLeftMotorTachoCount-lastLeftMotorTachoCount)/180;
    	distR = 3.14159*WHEEL_RAD*(currentRightMotorTachoCount-lastRightMotorTachoCount)/180;
    	lastLeftMotorTachoCount = currentLeftMotorTachoCount;
    	lastRightMotorTachoCount = currentRightMotorTachoCount;
    	deltaD = 0.5*(distL+distR);						//equivalent to deltaC, distance travelled by the center of the robot				// compute vehicle displacement	
    	deltaT = ((distL-distR)/TRACK)*(180/Math.PI);					//delta theta										// compute change in heading
    	theta += deltaT;																		// update heading
    	dX = deltaD * Math.sin(Math.toRadians(theta));															// compute X component of displacement  
    	dY = deltaD * Math.cos(Math.toRadians(theta)); 															// compute Y component of displacement 
    	x = x + dX;																				// update estimates of X and Y position 
    	y = y + dY;
    	// TODO Calculate new robot position based on tachometer counts
      
    	// TODO Update odometer values with new calculated values, eg
    	//odo.update(dx, dy, dtheta);

    	// this ensures that the odometer only runs once every period
    	updateEnd = System.currentTimeMillis();
    	if (updateEnd - updateStart < ODOMETER_PERIOD) {
    		try {
    			Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        	} catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }
  
  // IT IS NOT NECESSARY TO MODIFY ANYTHING BELOW THIS LINE
  
  /**
   * Returns the Odometer data.
   * <p>
   * Writes the current position and orientation of the robot onto the odoData array. {@code odoData[0] =
   * x, odoData[1] = y; odoData[2] = theta;}
   * 
   * @param position the array to store the odometer data
   * @return the odometer data.
   */
  public double[] getXYT() {
    double[] position = new double[3];
    lock.lock();
    try {
      while (isResetting) { // If a reset operation is being executed, wait until it is over.
        doneResetting.await(); // Using await() is lighter on the CPU than simple busy wait.
      }

      position[0] = x;
      position[1] = y;
      position[2] = theta;
    } catch (InterruptedException e) {
      e.printStackTrace();
    } finally {
      lock.unlock();
    }

    return position;
  }

  /**
   * Adds dx, dy and dtheta to the current values of x, y and theta, respectively. Useful for
   * odometry.
   * 
   * @param dx
   * @param dy
   * @param dtheta
   */
  public void update(double dx, double dy, double dtheta) {
    lock.lock();
    isResetting = true;
    try {
      x += dx;
      y += dy;
      theta = (theta + (360 + dtheta) % 360) % 360; // keeps the updates within 360 degrees
      isResetting = false;
      doneResetting.signalAll(); // Let the other threads know we are done resetting
    } finally {
      lock.unlock();
    }

  }
  public double getTheta() {
	    lock.lock();
	    isResetting = true;
	    try {
	      isResetting = false;
	      doneResetting.signalAll(); // Let the other threads know we are done resetting
	    } finally {
	      lock.unlock();
	    }
	    return theta;
	  }

  /**
   * Overrides the values of x, y and theta. Use for odometry correction.
   * 
   * @param x the value of x
   * @param y the value of y
   * @param theta the value of theta in degrees
   */
  public void setXYT(double x, double y, double theta) {
    lock.lock();
    isResetting = true;
    try {
      this.x = x;
      this.y = y;
      this.theta = theta;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overwrites x. Use for odometry correction.
   * 
   * @param x the value of x
   */
  public void setX(double x) {
    lock.lock();
    isResetting = true;
    try {
      this.x = x;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overwrites y. Use for odometry correction.
   * 
   * @param y the value of y
   */
  public void setY(double y) {
    lock.lock();
    isResetting = true;
    try {
      this.y = y;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overwrites theta. Use for odometry correction.
   * 
   * @param theta the value of theta
   */
  public void setTheta(double theta) {
    lock.lock();
    isResetting = true;
    try {
      this.theta = theta;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

}
