package net.jovacorp.bjo.bangbang;

import at.fhv.dgr1992.differentialWheels.Speed;
import at.fhv.dgr1992.ePuck.ePuckVRep.EPuckVRep;
import net.jovacorp.bjo.AbstractController;

public class PushController extends AbstractController {
  double maxVel = 120.0 * java.lang.Math.PI / 180.0; // 4/3 of a full wheel turn

  public static void main(String[] args) throws ControllerException {
    PushController pbcontroller = new PushController();
    pbcontroller.startBehavior();
  }

  @Override
  protected void act(EPuckVRep epuck) throws Exception {
    epuck.senseAllTogether();
    double[] distVector = epuck.getProximitySensorValues();

    if (!isObstacleInFront(distVector))
      // nothing in front, go ahead approaching a box
      epuck.setMotorSpeeds(new Speed(maxVel, maxVel));
    else {
      // if we detect something the numbers get smaller =>
      // if we detect something on the left but not the right =>
      // turn left
      if (distVector[0] + distVector[1] + distVector[2]
          < distVector[3] + distVector[4] + distVector[5])
        // turn counterclockwise
        epuck.setMotorSpeeds(new Speed((maxVel / 2.0), (maxVel / 2.0) + 1));
      else
        // turn clockwise
        epuck.setMotorSpeeds(new Speed((maxVel / 2.0) + 1, (maxVel / 2.0)));
    }

    epuck.stepsim(1);
  }

  @Override
  protected EPuckVRep setup() throws Exception {
    return baseSetup();
  }
}
