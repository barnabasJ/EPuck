package net.jovacorp.bjo.proportional.calculator;

import net.jovacorp.bjo.proportional.ImageToVectorMapper;
import at.fhv.dgr1992.differentialWheels.CameraImage;
import at.fhv.dgr1992.differentialWheels.Speed;
import at.fhv.dgr1992.ePuck.EPuck;
import at.fhv.dgr1992.exceptions.RobotFunctionCallException;
import at.fhv.dgr1992.exceptions.VelocityLimitException;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class ApproachCalculator {

    double maxVel = 120.0 * Math.PI / 180.0;  // 4/3 of a full wheel turn

    double[][] proportionalMatrixData = new double[][]{{0, 0.03}, {0.03, 0}};
    RealMatrix proportionalMatrix = MatrixUtils.createRealMatrix(proportionalMatrixData);

    double[] baseVelocity = new double[]{maxVel, maxVel};
    RealVector baseVelocityVector = MatrixUtils.createRealVector(baseVelocity);

    public void calculateSpeed(EPuck epuck, CameraImage image) throws VelocityLimitException, RobotFunctionCallException {

        double[] sensorValues = ImageToVectorMapper.mapImageToVector(image);
        double[] result = proportionalMatrix.operate(sensorValues);

        RealVector motorValues = MatrixUtils.createRealVector(result);
        motorValues = motorValues.add(baseVelocityVector);

        epuck.setMotorSpeeds(new Speed(motorValues.getEntry(0), motorValues.getEntry(1)));
    }

    public boolean activate(CameraImage image) {
        double[] sensorValues = ImageToVectorMapper.mapImageToVector(image);

        return sensorValues[1] > -32;
    }

}
