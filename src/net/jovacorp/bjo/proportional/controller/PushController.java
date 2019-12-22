package net.jovacorp.bjo.proportional.controller;

import at.fhv.dgr1992.ePuck.ePuckVRep.EPuckVRep;
import net.jovacorp.bjo.AbstractController;
import net.jovacorp.bjo.proportional.calculator.PushDiscCalculator;

public class PushController extends AbstractController {
  private PushDiscCalculator calculator = new PushDiscCalculator();

  public static void main(String[] args) throws ControllerException {
    PushController pbcontroller = new PushController();
    pbcontroller.startBehavior();
  }

  @Override
  protected void act(EPuckVRep epuck) throws Exception {
    epuck.senseAllTogether();
    calculator.calculateSpeed(epuck, epuck.getProximitySensorValues());
    epuck.stepsim(1);
  }

  @Override
  protected EPuckVRep setup() throws Exception {
    return baseSetup();
  }
}
