package net.jovacorp.bjo.bangbang;

import at.fhv.dgr1992.differentialWheels.Speed;
import at.fhv.dgr1992.ePuck.ePuckVRep.EPuckVRep;
import net.jovacorp.bjo.AbstractController;

import java.util.Arrays;

public class PushController extends AbstractController {
  // Barnabas: Camera properties -> don't change
  int resolX = 64, resolY = 64;
  double maxVel = 120.0 * java.lang.Math.PI / 180.0; // 4/3 of a full wheel turn
  double noDetectionDistance = 0.05;
  int stepCounter = 0;

  public static void main(String[] args) throws ControllerException {
    PushController pbcontroller = new PushController();
    pbcontroller.startBehavior();
  }

  @Override
  protected void act(EPuckVRep epuck) throws Exception {
    stepCounter += 1;
    // boolean newImage = false;
    epuck.senseAllTogether();
    double[] distVector = epuck.getProximitySensorValues();
    // double[] lightVector = epuck.getLightSensorValues(); // Anne: ist nicht implementiert bei
    // VRep??
    // Acceleration acceleration = epuck.getAccelerometerValues();

    /* // Anne: würde Bild einlesen, sollte man nicht zu oft machen weil TCP aufwendig
                if (stepCounter%4 == 0) {
                    try {
                        image = epuck.getCameraImage();
                    } catch (Exception e) {
                        System.out.println(e.toString());
                    }
                    newImage = true;
                }
    */

    if (vectorGreater(
        Arrays.copyOfRange(distVector, 0, 6),
        new double[] {
          0.25 * noDetectionDistance,
          0.25 * noDetectionDistance,
          0.25 * noDetectionDistance,
          0.25 * noDetectionDistance,
          0.25 * noDetectionDistance,
          0.25 * noDetectionDistance
        }))
      // nothing in front, go ahead approaching a box
      epuck.setMotorSpeeds(new Speed(maxVel, maxVel));
    else {
      // TODO check that distVector 0 == distVector 3 and so on to be really central
      // Obstacle in front
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
