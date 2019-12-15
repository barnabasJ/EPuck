package net.jovacorp.bjo;

import at.fhv.dgr1992.differentialWheels.Speed;
import at.fhv.dgr1992.ePuck.ePuckVRep.EPuckVRep;
import at.fhv.dgr1992.exceptions.RobotFunctionCallException;
import at.fhv.dgr1992.exceptions.VelocityLimitException;

public abstract class AbstractController {
  protected final double maxVel = 120.0 * Math.PI / 180.0; // 4/3 of a full wheel turn
  protected final double noDetectionDistance = 0.05;

  public void startBehavior() throws ControllerException {
    try {
      EPuckVRep epuck = setup();
      while (epuck.isConnected()) act(epuck);
    } catch (Exception e) {
      throw new ControllerException(e);
    }
  }

  protected abstract void act(EPuckVRep epuck) throws Exception;

  protected abstract EPuckVRep setup() throws Exception;

  protected EPuckVRep newSynchronousEPuck() {
    return new EPuckVRep("ePuck", "127.0.0.1", 19999, true);
  }

  protected EPuckVRep sensorSetup(EPuckVRep epuck)
      throws VelocityLimitException, RobotFunctionCallException {
    epuck.enableAllSensors();
    epuck.enableCamera();
    // epuck.enablePose();   //in all exercises, you are not allowed to use this sensor
    epuck.setSenseAllTogether();
    epuck.setMotorSpeeds(new Speed(0, 0));
    return epuck;
  }

  protected EPuckVRep baseSetup() throws Exception {
    EPuckVRep ePuckVRep = newSynchronousEPuck();
    if (!ePuckVRep.isConnected()) ePuckVRep.connect();
    sensorSetup(ePuckVRep);
    ePuckVRep.startsim();
    return ePuckVRep;
  }

  protected boolean vectorGreater(double[] v1, double[] v2) {
    // compare two vector element-wise
    if (v1.length != v2.length) {
      System.err.println("tries to compare two vectors of different lengths");
      System.exit(1);
    }
    for (int i = 0; i < v1.length; i++) if (v1[i] <= v2[i]) return false;
    return true;
  }

  protected boolean isObstacleInFrontOfSensors(double[] distances, double distanceModificator) {
    double[] detectionVector = new double[distances.length];
    for (int i = 0; i < distances.length; i++)
      detectionVector[i] = distanceModificator * noDetectionDistance;
    return !vectorGreater(distances, detectionVector);
  }

  protected int[] createRange(int start, int end) {
    int[] range = new int[end - start];
    for (int i = 0; i < end - start; i++) range[i] = start + i;
    return range;
  }

  protected double[] extractValues(double[] values, int... indices) {
    double[] extractedValues = new double[indices.length];

    for (int i = 0; i < indices.length; i++) {
      extractedValues[i] = values[indices[i]];
    }
    return extractedValues;
  }

  public class ControllerException extends Exception {
    public ControllerException(Exception e) {
      super(e);
    }
  }
}
