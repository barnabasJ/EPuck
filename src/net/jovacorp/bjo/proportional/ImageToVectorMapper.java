package net.jovacorp.bjo.proportional;

import at.fhv.dgr1992.differentialWheels.CameraImage;
import at.fhv.dgr1992.differentialWheels.CameraImagePixel;

public class ImageToVectorMapper {

  private static double deviation;

  public static double[] mapImageToVector(CameraImage image) {

    int height = image.getBufferedImage().getHeight();
    int width = image.getBufferedImage().getWidth();
    boolean first = true;
    int bottomRight = 0;
    int topLeft = 0;

    for (int x = 0; x < width; x++) {
      for (int y = 0; y < height; y++) {
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
    int imageCentre = width / 2;
    double doorCentroid = (bottomRight + topLeft) / 2.0;

    deviation =
        (imageCentre - doorCentroid) / 15.0; // deviation from camera centre to object centre

    if (doorCentroid == 0) {
      return new double[] {-4, 0};
    }

    if (deviation < 0) {
      return new double[] {deviation, 0};
    }

    return new double[] {0, -deviation};
  }
}
