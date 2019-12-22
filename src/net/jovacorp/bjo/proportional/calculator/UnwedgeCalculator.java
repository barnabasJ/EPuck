package net.jovacorp.bjo.proportional.calculator;

import at.fhv.dgr1992.differentialWheels.Acceleration;
import at.fhv.dgr1992.differentialWheels.Speed;
import at.fhv.dgr1992.ePuck.EPuck;
import at.fhv.dgr1992.exceptions.RobotFunctionCallException;
import at.fhv.dgr1992.exceptions.VelocityLimitException;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class UnwedgeCalculator {

  double maxVel = 120.0 * Math.PI / 180.0; // 4/3 of a full wheel turn

  double[][] proportionalMatrixData = new double[][] {{0, 0.001}, {0.001, 0}};
  RealMatrix proportionalMatrix = MatrixUtils.createRealMatrix(proportionalMatrixData);

  double[] baseVelocity = new double[] {maxVel, 0};
  RealVector baseVelocityVector = MatrixUtils.createRealVector(baseVelocity);

  public void calculateSpeed(EPuck epuck)
      throws VelocityLimitException, RobotFunctionCallException {

    double[] sensorValues = new double[] {-1, -1};
    double[] result = proportionalMatrix.operate(sensorValues);

    RealVector motorValues = MatrixUtils.createRealVector(result);
    motorValues = motorValues.add(baseVelocityVector);

    //        epuck.setMotorSpeeds(new Speed(motorValues.getEntry(0), motorValues.getEntry(1)));
    epuck.setMotorSpeeds(new Speed(maxVel, -maxVel));
  }

  public boolean activate(Acceleration acceleration, boolean blocked) {

    double thresholdStop = 0.9;
    double forwardAcceleration = acceleration.getX();

    boolean stopped = (forwardAcceleration > 0.05);

    return stopped || blocked;
  }
}
