package net.jovacorp.bjo.proportional;

import at.fhv.dgr1992.differentialWheels.CameraImage;
import at.fhv.dgr1992.differentialWheels.CameraImagePixel;

public class ImageToVectorMapper {

  public static double[] mapImageToVector(CameraImage image) {

    int height = image.getBufferedImage().getHeight();
    int width = image.getBufferedImage().getWidth();

    int sumX = 0;
    int countX = 0;

    for (int x = 0; x < width; x++) {
      for (int y = 0; y < height; y++) {
        CameraImagePixel pixel = image.getPixel(x, y);
        int r = pixel.getRed();
        int g = pixel.getGreen();
        int b = pixel.getBlue();

        if ((r + b + g) == 0) {
          sumX += x;
          countX++;
        }
      }
    }

    int imageCentre = width / 2;
    int doorCentroid = countX == 0 ? 0 : sumX / countX;

    int deviation = imageCentre - doorCentroid; // deviation from camera centre to object centre

    if (deviation > 0) {
      return new double[] {0, -deviation};
    }

    return new double[] {deviation, 0};
  }
}
