package net.jovacorp.bjo.bangbang;

import at.fhv.dgr1992.differentialWheels.Speed;
import at.fhv.dgr1992.ePuck.ePuckVRep.EPuckVRep;
import at.fhv.dgr1992.exceptions.RobotFunctionCallException;
import at.fhv.dgr1992.exceptions.VelocityLimitException;
import net.jovacorp.bjo.AbstractController;

/**
 * A Controller which lets the epuck follow the wall on it's left side.
 * At the start the epuck will drive forward until it detects a wall.
 */
public class WallController extends AbstractController {
  private boolean wallFound = false;
  private Status lastStatus;
  private int timesInSameStatus = 1;

  public static void main(String[] args) throws ControllerException {
    WallController pbcontroller = new WallController();
    pbcontroller.startBehavior();
  }

  @Override
  protected void act(EPuckVRep epuck) throws Exception {
    epuck.senseAllTogether();
    double[] distVector = epuck.getProximitySensorValues();

    if (!wallFound) {
      wallFound = findWall(epuck, distVector);
    } else {
      followWall(epuck, distVector);
    }
    epuck.stepsim(1);
  }

  /**
   * Lets the epuck follow the wall by calculating it's current status
   * and acting accordingly
   * @param epuck the epuck to guide
   * @param distVector the sensor values of the epuck
   * @throws VelocityLimitException
   * @throws RobotFunctionCallException
   */
  private void followWall(EPuckVRep epuck, double[] distVector)
      throws VelocityLimitException, RobotFunctionCallException {
    Status currentStatus = calcCurrentStatus(distVector);
    timesInSameStatus = lastStatus == currentStatus ? timesInSameStatus + 1 : 1;
    switch (currentStatus) {
      case WALL_IN_FRONT:
        // if something is in front of us -> turn clockwise
        System.out.println("Wall in front");
        epuck.setMotorSpeeds(new Speed(maxVel, calcSlowWheelSpeed()));
        break;
      case TO_CLOSE_TO_WALL:
        // if we're to close to the wall ->  turn clockwise
        System.out.println("To close to wall");
        epuck.setMotorSpeeds(new Speed(maxVel, calcSlowWheelSpeed()));
        break;
      case NO_WALL_TO_THE_LEFT:
        // if we don't detect the wall on the left any more -> turn counterclockwise
        System.out.println("No Wall to the left");
        System.out.println(timesInSameStatus);
        if (timesInSameStatus > 100) wallFound = false;
        epuck.setMotorSpeeds(new Speed(0, maxVel));
        break;
      case OK:
        // otherwise -> straight
        System.out.println("Follow straight");
        epuck.setMotorSpeeds(new Speed(maxVel, maxVel));
    }
    lastStatus = currentStatus;
  }

  /**
   * Calculates the speed for the slower wheel for a turn.
   * It takes into account how long the epuck is already inside
   * the same status. If it takes to long, the epuck is probably to close
   * to an obstacle and needs to turn on the spot.
   * @return the speed for the slower wheel in a turn
   */
    private double calcSlowWheelSpeed() {
        double v = timesInSameStatus <= 50 ? maxVel / (10 * timesInSameStatus) : -1.0;
        System.out.println(v);
        return v;
    }

  /**
   * Calculates the current status of the epuck
   * @param distVector the sensor values of the epuck
   * @return the calculated status
   * @throws VelocityLimitException
   * @throws RobotFunctionCallException
   */
    private Status calcCurrentStatus(double[] distVector)
      throws VelocityLimitException, RobotFunctionCallException {
    if (isObstacleInFrontOfSensors(extractValues(distVector, 2, 3), 1)) {
      // if something is in front of us -> turn clockwise
      return Status.WALL_IN_FRONT;
    } else if (isObstacleInFrontOfSensors(extractValues(distVector, 1), 0.25)) {
      // if we're to close to the wall ->  turn clockwise
      return Status.TO_CLOSE_TO_WALL;
    } else if (!isObstacleInFrontOfSensors(extractValues(distVector, 0, 1, 2), 1)) {
      // if we don't detect the wall on the left any more -> turn counterclockwise
      return Status.NO_WALL_TO_THE_LEFT;
    }
    return Status.OK;
  }

  /**
   * Lets the epuck find the wall by driving forward until it detects an obstacle
   * After an obstacle is detected it will turn the epuck clockwise to get the wall
   * to the left side of the epuck
   * @param epuck the epuck to guide
   * @param distVector the sensor values of the epuck
   * @return true if a wall is detected otherwise false
   * @throws VelocityLimitException
   * @throws RobotFunctionCallException
   */
  private boolean findWall(EPuckVRep epuck, double[] distVector)
      throws VelocityLimitException, RobotFunctionCallException {
    if (!isObstacleInFrontOfSensors(extractValues(distVector, 1, 2), 1)) {
      // not close to any wall on left side -> drive straight to find wall
      epuck.setMotorSpeeds(new Speed(maxVel, maxVel));
      return false;
    }
    // wall found on left side, turn clockwise
    epuck.setMotorSpeeds(new Speed((maxVel), (maxVel / 10.0)));
    if (isObstacleInFrontOfSensors(extractValues(distVector, 1, 2), 0.3)) return true;
    return false;
  }

  @Override
  protected EPuckVRep setup() throws Exception {
    return baseSetup();
  }

  /**
   * The different statuses the epuck can be in
   */
  private enum Status {
    WALL_IN_FRONT,
    TO_CLOSE_TO_WALL,
    NO_WALL_TO_THE_LEFT,
    OK,
  }
}
