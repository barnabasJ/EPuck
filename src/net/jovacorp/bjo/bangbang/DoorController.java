package net.jovacorp.bjo.bangbang;

import at.fhv.dgr1992.differentialWheels.CameraImage;
import at.fhv.dgr1992.differentialWheels.CameraImagePixel;
import at.fhv.dgr1992.differentialWheels.Speed;
import at.fhv.dgr1992.ePuck.ePuckVRep.EPuckVRep;
import at.fhv.dgr1992.ePuck.ePuckVRep.exceptions.StepSimNotPossibleException;
import at.fhv.dgr1992.ePuck.ePuckVRep.exceptions.SynchrounusModeNotActivatedException;
import at.fhv.dgr1992.exceptions.RobotFunctionCallException;
import at.fhv.dgr1992.exceptions.SensorNotEnabledException;
import at.fhv.dgr1992.exceptions.VelocityLimitException;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import java.util.Arrays;

public class DoorController {

    // Anne: Camera properties -> don't change
    int resolX = 64, resolY = 64;
    double maxVel = 120.0 * Math.PI / 180.0;  // 4/3 of a full wheel turn
    double noDetectionDistance = 0.05;

    //the next three declarations are just example code, not used in this behavior
    double[][] proportionalMatrixData = new double[][]{{0, 0, 0, 0}, {0, 0, 0, 0}};
    RealMatrix proportionalMatrix = MatrixUtils.createRealMatrix(proportionalMatrixData);
    double[] baseVelocity = new double[]{maxVel / 6.0, maxVel / 6.0};


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
            epuck.enableCamera();
            //epuck.enablePose();   //in all exercises, you are not allowed to use this sensor
            epuck.setSenseAllTogether();
            epuck.setMotorSpeeds(new Speed(0, 0));
            if (synchron)
                epuck.startsim();
            int stepCounter = 0;

            CameraImage image = new CameraImage(resolX, resolY);

            while (epuck.isConnected()) {
                stepCounter += 1;
                boolean newImage = false;
                epuck.senseAllTogether();
                double[] distVector = epuck.getProximitySensorValues();
                //double[] lightVector = epuck.getLightSensorValues(); // Anne: ist nicht implementiert bei VRep??
                //Acceleration acceleration = epuck.getAccelerometerValues();

                // Anne: Bild einlesen, sollte man nicht zu oft machen weil TCP aufwendig
                if (stepCounter % 4 == 0) {
                    try {
                        image = epuck.getCameraImage();
                    } catch (Exception e) {
                        System.out.println(e.toString());
                    }
                    newImage = true;
                }

                int heigth = image.getBufferedImage().getHeight();
                int width = image.getBufferedImage().getWidth();

                boolean first = true;
                Point topLeft = null;
                Point bottomRight = null;

                for (int x = 0; x < width; x++) {
                    for (int y = 0; y < heigth; y++) {
                        CameraImagePixel pixel = image.getPixel(x, y);
                        int r = pixel.getRed();
                        int g = pixel.getGreen();
                        int b = pixel.getBlue();

                        if ((r + b + g) == 0) {
                            if (first) {
                                first = false;
                                topLeft = new Point(x, y);
                            } else {
                                bottomRight = new Point(x, y);
                            }
                        }
                    }
                }

                int imageCentre = width / 2;
                int doorCentre = (bottomRight.getX() + topLeft.getX()) / 2;

                if (!vectorGreater(Arrays.copyOfRange(distVector, 0, 6), new double[]{0.9 * noDetectionDistance, 0.9 * noDetectionDistance, 0.9 * noDetectionDistance, 0.9 * noDetectionDistance,
                        0.9 * noDetectionDistance, 0.9 * noDetectionDistance}))
                    // stop
                    epuck.setMotorSpeeds(new Speed(0, 0));
                else {
                    if (imageCentre > doorCentre)
                        //turn counterclockwise
                        epuck.setMotorSpeeds(new Speed((maxVel / 1.1), (maxVel)));
                    else if (imageCentre < doorCentre)
                        //turn clockwise
                        epuck.setMotorSpeeds(new Speed((maxVel), (maxVel / 1.1)));
                    else
                        //drive straight
                        epuck.setMotorSpeeds(new Speed(maxVel, maxVel));
                }

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
        DoorController pbcontroller = new DoorController();
        pbcontroller.startBehavior();
    }
}
