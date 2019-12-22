package net.jovacorp.bjo.proportional.controller;

import at.fhv.dgr1992.differentialWheels.Speed;
import at.fhv.dgr1992.ePuck.ePuckVRep.EPuckVRep;
import net.jovacorp.bjo.AbstractController;
import net.jovacorp.bjo.proportional.calculator.WallCalculator;

public class WallController extends AbstractController {
  private WallCalculator calculator = new WallCalculator();
  private boolean wallFound = false;

  public static void main(String[] args) throws ControllerException {
    WallController pbcontroller = new WallController();
    pbcontroller.startBehavior();
  }

  @Override
  protected void act(EPuckVRep epuck) throws Exception {
    epuck.senseAllTogether();

    if (!wallFound) {
      epuck.setMotorSpeeds(new Speed(maxVel, maxVel));
      if (any(extractValues(epuck.getProximitySensorValues(), 2, 3), d -> d < noDetectionDistance))
        wallFound = true;
    } else {
      double[] proximitySensorValues = epuck.getProximitySensorValues();
      double[] doubles = extractValues(proximitySensorValues, 1, 2, 3, 4);
      double[] scaledSensorValues =
          scaleSensorValues(
              doubles,
              p -> {
                System.out.println(p.second % 2);
                return p.second % 2 == 0
                    ? p.first < noDetectionDistance ? 1.0 : 0
                    : p.first > noDetectionDistance ? 0 : 1.0;
              });
      Speed speed = calculator.calculateSpeed(scaledSensorValues);
      printSpeed(speed);
      epuck.setMotorSpeeds(speed);
    }
    epuck.stepsim(1);
  }

  @Override
  protected EPuckVRep setup() throws Exception {
    return baseSetup();
  }
}
