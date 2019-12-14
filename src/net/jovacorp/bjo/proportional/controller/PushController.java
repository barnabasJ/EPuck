package net.jovacorp.bjo.proportional.controller;

import net.jovacorp.bjo.proportional.calculator.PushDiscCalculator;
import at.fhv.dgr1992.differentialWheels.Speed;
import at.fhv.dgr1992.ePuck.ePuckVRep.EPuckVRep;
import at.fhv.dgr1992.ePuck.ePuckVRep.exceptions.StepSimNotPossibleException;
import at.fhv.dgr1992.ePuck.ePuckVRep.exceptions.SynchrounusModeNotActivatedException;
import at.fhv.dgr1992.exceptions.RobotFunctionCallException;
import at.fhv.dgr1992.exceptions.SensorNotEnabledException;
import at.fhv.dgr1992.exceptions.VelocityLimitException;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

public class PushController {

    // Anne: Camera properties -> don't change
    int resolX = 64, resolY = 64;
    double maxVel = 120.0 * Math.PI / 180.0;  // 4/3 of a full wheel turn
    double noDetectionDistance = 0.05;

    //the next three declarations are just example code, not used in this behavior
    double[][] proportionalMatrixData = new double[][]{{0, 0, 0, -1, -5, -30}, {-30, -5, -1, 0, 0, 0}};
    RealMatrix proportionalMatrix = MatrixUtils.createRealMatrix(proportionalMatrixData);
    double[] baseVelocity = new double[]{maxVel, maxVel};

    boolean vectorGreater(double[] v1, double[] v2) {
        //compare two vector element-wise
        if (v1.length != v2.length) {
            System.err.println("tries to compare two vectors of different lengths");
            System.exit(1);
        }
        for (int i = 0; i < v1.length; i++)
            if (v1[i] <= v2[i])
                return false;
        return true;
    }

    void startBehavior() {

        boolean synchron = true;

        EPuckVRep epuck = new EPuckVRep("ePuck", "127.0.0.1", 19999, synchron);

        try {
            if (!epuck.isConnected()) {
                epuck.connect();
            }
            epuck.enableAllSensors();
            //epuck.enableCamera();
            //epuck.enablePose();   //in all exercises, you are not allowed to use this sensor
            epuck.setSenseAllTogether();
            epuck.setMotorSpeeds(new Speed(0, 0));
            if (synchron)
                epuck.startsim();
            int stepCounter = 0;

            //CameraImage image = new CameraImage(resolX,resolY);

            PushDiscCalculator calculator = new PushDiscCalculator();

            while (epuck.isConnected()) {
                stepCounter += 1;
                //boolean newImage = false;
                epuck.senseAllTogether();
                double[] distVector = epuck.getProximitySensorValues();
                //double[] lightVector = epuck.getLightSensorValues(); // Anne: ist nicht implementiert bei VRep??
                //Acceleration acceleration = epuck.getAccelerometerValues();

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

                calculator.calculateSpeed(epuck, distVector);

                if (synchron)
                    epuck.stepsim(1);
                else
                    Thread.sleep(50);
            }

        } catch (VelocityLimitException e) {
            e.printStackTrace();
        } catch (RobotFunctionCallException e) {
            e.printStackTrace();
        } catch (SensorNotEnabledException e) {
            e.printStackTrace();
        } catch (SynchrounusModeNotActivatedException e) {
            e.printStackTrace();
        } catch (StepSimNotPossibleException e) {
            e.printStackTrace();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public static void main(String[] args) {
        PushController pbcontroller = new PushController();
        pbcontroller.startBehavior();
    }
}
