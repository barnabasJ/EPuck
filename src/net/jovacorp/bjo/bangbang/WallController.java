package net.jovacorp.bjo.bangbang;

import at.fhv.dgr1992.differentialWheels.Speed;
import at.fhv.dgr1992.ePuck.ePuckVRep.EPuckVRep;
import at.fhv.dgr1992.exceptions.RobotFunctionCallException;
import at.fhv.dgr1992.exceptions.VelocityLimitException;
import net.jovacorp.bjo.AbstractController;

public class WallController extends AbstractController {
  private boolean wallFound = false;

  public static void main(String[] args) throws ControllerException {
    WallController pbcontroller = new WallController();
    pbcontroller.startBehavior();
  }

  @Override
  protected void act(EPuckVRep epuck) throws Exception {
    epuck.senseAllTogether();
    double[] distVector = epuck.getProximitySensorValues();
    for (double dist : distVector) {
      System.out.print(dist);
      System.out.print(" : ");
    }
    System.out.println("-");

    if (!wallFound) {
      wallFound = findWall(epuck, distVector);
    } else {
      followWall(epuck, distVector);
    }
    epuck.stepsim(1);
  }

  private void followWall(EPuckVRep epuck, double[] distVector)
      throws VelocityLimitException, RobotFunctionCallException {
    if (distVector[2] < noDetectionDistance || distVector[3] < noDetectionDistance) {
      // front sensor < noDetectionDistance -> turn clockwise
      epuck.setMotorSpeeds(new Speed((maxVel), (maxVel / 10.0)));
    } else if (distVector[1] < 0.25 * noDetectionDistance) {
      // clockwise (to close to wall)
      epuck.setMotorSpeeds(new Speed((maxVel), (maxVel / 5.0)));
    } else if (distVector[1] > noDetectionDistance && distVector[2] > noDetectionDistance) {
      // counterclockwise (to far away from wall)
      epuck.setMotorSpeeds(new Speed((maxVel / 2), maxVel));
    } else {
      // straight
      epuck.setMotorSpeeds(new Speed(maxVel, maxVel));
    }
  }

  private boolean findWall(EPuckVRep epuck, double[] distVector)
      throws VelocityLimitException, RobotFunctionCallException {
    if (!isObstacleInFrontOfSensors(extractValues(distVector, 1, 2), 1)) {
      // not close to any wall on left side -> drive straight to find wall
      epuck.setMotorSpeeds(new Speed(maxVel, maxVel));
      return false;
    }
    // wall found on left side, turn clockwise
    epuck.setMotorSpeeds(new Speed((maxVel), (maxVel / 10.0)));
    if (isObstacleInFrontOfSensors(extractValues(distVector, 1,2), 0.3))
      return true;
    return false;
  }

  @Override
  protected EPuckVRep setup() throws Exception {
    return baseSetup();
  }
}
