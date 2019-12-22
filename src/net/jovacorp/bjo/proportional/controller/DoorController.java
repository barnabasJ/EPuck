package net.jovacorp.bjo.proportional.controller;

import at.fhv.dgr1992.differentialWheels.Acceleration;
import at.fhv.dgr1992.differentialWheels.CameraImage;
import at.fhv.dgr1992.ePuck.ePuckVRep.EPuckVRep;
import net.jovacorp.bjo.AbstractController;
import net.jovacorp.bjo.proportional.calculator.ApproachCalculator;

public class DoorController extends AbstractController {
  private ApproachCalculator calculator = new ApproachCalculator();
  private CameraImage image;
  private int stepCounter = 0;

  public static void main(String[] args) throws ControllerException {
    DoorController pbcontroller = new DoorController();
    pbcontroller.startBehavior();
  }

  @Override
  protected void act(EPuckVRep epuck) throws Exception {
    epuck.senseAllTogether();
    Acceleration acceleration = epuck.getAccelerometerValues();

    // Bild einlesen, sollte man nicht zu oft machen weil TCP aufwendig
    if (stepCounter % 4 == 0) {
      image = epuck.getCameraImage();
    }

    System.out.println(
        acceleration.getX() + " : " + acceleration.getY() + " : " + acceleration.getZ());

    calculator.calculateSpeed(epuck, image);
    epuck.stepsim(1);
    stepCounter += 1;
  }

  @Override
  protected EPuckVRep setup() throws Exception {
    return baseSetup();
  }
}
