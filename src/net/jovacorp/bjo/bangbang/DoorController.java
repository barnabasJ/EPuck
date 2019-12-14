package net.jovacorp.bjo.bangbang;

import at.fhv.dgr1992.differentialWheels.CameraImage;
import at.fhv.dgr1992.differentialWheels.CameraImagePixel;
import at.fhv.dgr1992.differentialWheels.Speed;
import at.fhv.dgr1992.ePuck.ePuckVRep.EPuckVRep;
import net.jovacorp.bjo.AbstractController;

public class DoorController extends AbstractController {
  private CameraImage image;
  private int stepCounter = 0;
  private int doorCenter;
  private int imageCenter;

  public static void main(String[] args) throws ControllerException {
    DoorController pbcontroller = new DoorController();
    pbcontroller.startBehavior();
  }

  @Override
  protected void act(EPuckVRep epuck) throws Exception {
    stepCounter += 1;
    epuck.senseAllTogether();
    double[] distVector = epuck.getProximitySensorValues();

    // Bild einlesen, sollte man nicht zu oft machen weil TCP aufwendig
    if (stepCounter % 4 == 0) {
      image = epuck.getCameraImage();
      doorCenter = findDoorCenter();
      imageCenter = image.getBufferedImage().getWidth() / 2;
    }

    if (imageCenter > doorCenter) {
      // turn counterclockwise
      System.out.println("left");
      epuck.setMotorSpeeds(new Speed((maxVel / 1.1), (maxVel)));
    } else if (imageCenter < doorCenter) {
      // turn clockwise
      System.out.println("right");
      epuck.setMotorSpeeds(new Speed((maxVel), (maxVel / 1.1)));
    } else {
      // drive straight
      System.out.println("straight");
      if (isObstacleInFront(distVector, 0.9)) {
        // stop
        System.out.println("stop");
        epuck.setMotorSpeeds(new Speed(0, 0));
      } else {
        epuck.setMotorSpeeds(new Speed(maxVel, maxVel));
      }
    }
    epuck.stepsim(1);
  }

  private int findDoorCenter() {
    int heigth = image.getBufferedImage().getHeight();
    int width = image.getBufferedImage().getWidth();

    boolean first = true;
    int topLeft = -1;
    int bottomRight = -1;

    for (int x = 0; x < width; x++) {
      for (int y = 0; y < heigth; y++) {
        CameraImagePixel pixel = image.getPixel(x, y);
        int r = pixel.getRed();
        int g = pixel.getGreen();
        int b = pixel.getBlue();

        if ((r + b + g) == 0) {
          if (first) {
            first = false;
            topLeft = x;
          } else {
            bottomRight = x;
          }
        }
      }
    }

    return (bottomRight + topLeft) / 2;
  }

  @Override
  protected EPuckVRep setup() throws Exception {
    EPuckVRep ePuckVRep = baseSetup();
    image = ePuckVRep.getCameraImage();
    doorCenter = findDoorCenter();
    imageCenter = image.getBufferedImage().getWidth() / 2;
    return ePuckVRep;
  }
}
