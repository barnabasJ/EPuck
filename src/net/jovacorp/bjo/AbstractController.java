package net.jovacorp.bjo;

import at.fhv.dgr1992.differentialWheels.Speed;
import at.fhv.dgr1992.ePuck.ePuckVRep.EPuckVRep;
import at.fhv.dgr1992.exceptions.RobotFunctionCallException;
import at.fhv.dgr1992.exceptions.VelocityLimitException;

/**
 * This is the Base Controller class. It handles the normal sequence which is first setup and then
 * acting as long as the connection to the simulation is given.
 */
public abstract class AbstractController {
  protected final double maxVel = 120.0 * Math.PI / 180.0; // 4/3 of a full wheel turn
  protected final double noDetectionDistance = 0.05;

  /**
   * Starts the controller
   *
   * @throws ControllerException
   */
  public void startBehavior() throws ControllerException {
    try {
      EPuckVRep epuck = setup();
      while (epuck.isConnected()) act(epuck);
    } catch (Exception e) {
      throw new ControllerException(e);
    }
  }

  /**
   * This method must be implemented by every Controller, It is responsible for reading sensor data
   * and setting motor speeds and so on.
   *
   * @param epuck the set up epuck
   * @throws Exception
   */
  protected abstract void act(EPuckVRep epuck) throws Exception;

  /**
   * This method must be implemented by every Controller. It is responsible for setting up the epuck
   * for later use.
   *
   * @return the configured epuck instance
   * @throws Exception
   */
  protected abstract EPuckVRep setup() throws Exception;

  /**
   * Returns a new Synchronous Epuck instance
   *
   * @return a new Synchronous Epuck instance
   */
  protected EPuckVRep newSynchronousEPuck() {
    return new EPuckVRep("ePuck", "127.0.0.1", 19999, true);
  }

  /**
   * Enables all sensors
   *
   * @param epuck to configure
   * @return epuck with sensors enabled
   * @throws VelocityLimitException
   * @throws RobotFunctionCallException
   */
  protected EPuckVRep sensorSetup(EPuckVRep epuck)
      throws VelocityLimitException, RobotFunctionCallException {
    epuck.enableAllSensors();
    epuck.enableCamera();
    // epuck.enablePose();   //in all exercises, you are not allowed to use this sensor
    epuck.setSenseAllTogether();
    epuck.setMotorSpeeds(new Speed(0, 0));
    return epuck;
  }

  /**
   * A base setup that can be used by other Controllers
   *
   * @return an epuck instance with all sensors enabled and an established connection.
   * @throws Exception
   */
  protected EPuckVRep baseSetup() throws Exception {
    EPuckVRep ePuckVRep = newSynchronousEPuck();
    if (!ePuckVRep.isConnected()) ePuckVRep.connect();
    sensorSetup(ePuckVRep);
    ePuckVRep.startsim();
    return ePuckVRep;
  }

  /**
   * Checks if all values in the second vector are smaller than in the first
   *
   * @param v1 first vector
   * @param v2 second vector
   * @return true if all values in the second sector are smaller otherwise false
   */
  protected boolean vectorGreater(double[] v1, double[] v2) {
    // compare two vector element-wise
    if (v1.length != v2.length) {
      throw new IllegalStateException("tries to compare two vectors of different lengths");
    }
    for (int i = 0; i < v1.length; i++) if (v1[i] <= v2[i]) return false;
    return true;
  }

  /**
   * Checks if an obstacle is in front of the given sensor values
   *
   * @param distances the sensor values
   * @param distanceModificator the modificator with which the value of no detection is multiplied
   *     before comparison
   * @return true if a sensor is seeing an obstacle otherwise false
   */
  protected boolean isObstacleInFrontOfSensors(double[] distances, double distanceModificator) {
    double[] detectionVector = new double[distances.length];
    for (int i = 0; i < distances.length; i++)
      detectionVector[i] = distanceModificator * noDetectionDistance;
    return !vectorGreater(distances, detectionVector);
  }

  /**
   * Creates an array with all values from start to end with step 1
   *
   * @param start start value
   * @param end end value
   * @return int[] with all values from start to end
   */
  protected int[] createRange(int start, int end) {
    int[] range = new int[end - start];
    for (int i = 0; i < end - start; i++) range[i] = start + i;
    return range;
  }

  /**
   * Extracts the values at the given indicies from an array
   *
   * @param values The array with all values
   * @param indices The indicies of the values which should be extracted
   * @return the array with all extracted values
   */
  protected double[] extractValues(double[] values, int... indices) {
    double[] extractedValues = new double[indices.length];

    for (int i = 0; i < indices.length; i++) {
      extractedValues[i] = values[indices[i]];
    }
    return extractedValues;
  }

  protected double[] scaleSensorValues(double[] sensorValues) {
    double[] scaledSensorValues = new double[sensorValues.length];
    for (int i = 0; i < sensorValues.length; i++) {
      scaledSensorValues[i] = sensorValues[i] < noDetectionDistance ? 1.0 : 0.0;
    }
    return scaledSensorValues;
  }

  /** Wrapper Exception for the Exceptions thrown inside the Controller */
  public class ControllerException extends Exception {
    public ControllerException(Exception e) {
      super(e);
    }
  }
}
