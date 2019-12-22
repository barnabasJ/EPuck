package net.jovacorp.bjo.proportional.calculator;

import at.fhv.dgr1992.differentialWheels.Speed;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public abstract class AbstractCalculator {
  protected final double maxVel = 120.0 * Math.PI / 180.0; // 4/3 of a full wheel turn
  protected final double[][] proportionalMatrixData;
  protected final RealMatrix proportionalMatrix;
  protected double[] baseVelocity = new double[] {maxVel, maxVel};
  protected final RealVector baseVelocityVector = MatrixUtils.createRealVector(baseVelocity);

  public AbstractCalculator(double[][] popoMatrix) {
    this.proportionalMatrixData = popoMatrix;
    this.proportionalMatrix = MatrixUtils.createRealMatrix(proportionalMatrixData);
  }

  public abstract Speed calculateSpeed(double[] sensorValues);

  public abstract boolean activate(double[] sensorValues);
}
