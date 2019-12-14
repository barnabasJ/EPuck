package net.jovacorp.bjo.proportional.calculator;

import at.fhv.dgr1992.differentialWheels.Acceleration;
import at.fhv.dgr1992.differentialWheels.Speed;
import at.fhv.dgr1992.ePuck.EPuck;
import at.fhv.dgr1992.exceptions.RobotFunctionCallException;
import at.fhv.dgr1992.exceptions.VelocityLimitException;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.util.Arrays;

public class PushDiscCalculator {
    double maxVel = 120.0 * Math.PI / 180.0;  // 4/3 of a full wheel turn
    double[][] proportionalMatrixData = new double[][]{{0, 0, 0, -1, -10, -30}, {-30, -10, -1, 0, 0, 0}};
    RealMatrix proportionalMatrix = MatrixUtils.createRealMatrix(proportionalMatrixData);
    double[] baseVelocity = new double[]{maxVel * 1.5, maxVel * 1.5};
    RealVector baseVelocityVector = MatrixUtils.createRealVector(baseVelocity);

    public void calculateSpeed(EPuck epuck, double[] distVector) throws VelocityLimitException, RobotFunctionCallException {
        double[] sensorValues = Arrays.copyOfRange(distVector, 0, 6);
        double[] result = proportionalMatrix.operate(sensorValues);

        RealVector motorValues = MatrixUtils.createRealVector(result);
        motorValues = motorValues.add(baseVelocityVector);

        System.out.println(motorValues.getEntry(0) + " : " + motorValues.getEntry(1) + " - motor");

        epuck.setMotorSpeeds(new Speed(motorValues.getEntry(0), motorValues.getEntry(1)));
    }

    public boolean activate(double[] distVector, double noDetectionDistance) {
        return distVector[2] < 0.25 * noDetectionDistance || distVector[3] < 0.25 * noDetectionDistance;
    }
}
