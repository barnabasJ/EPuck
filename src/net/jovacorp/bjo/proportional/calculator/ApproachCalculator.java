package net.jovacorp.bjo.proportional.calculator;

import at.fhv.dgr1992.differentialWheels.Speed;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealVector;

public class ApproachCalculator extends AbstractCalculator {
  public ApproachCalculator() {
    super(new double[][] {{0, 0.5}, {0.5, 0}});
  }

  public Speed calculateSpeed(double[] sensorValues) {
    double[] result = proportionalMatrix.operate(sensorValues);
    RealVector motorValues = MatrixUtils.createRealVector(result);
    motorValues = motorValues.add(baseVelocityVector);

    return new Speed(motorValues.getEntry(0), motorValues.getEntry(1));
  }

  public boolean activate(double[] sensorValues) {
    return sensorValues[1] > -32;
  }
}
