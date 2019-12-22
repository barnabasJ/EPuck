package net.jovacorp.bjo.proportional.calculator;

import at.fhv.dgr1992.differentialWheels.Speed;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class PushDiscCalculatorTest {
  private final PushDiscCalculator pdc = new PushDiscCalculator();

  @Test
  public void puckInFrontTest() {
    Speed calculateSpeed = pdc.calculateSpeed(new double[] {0, 0, 1, 1, 0, 0});
    assertEquals(2.0, calculateSpeed.getLeft(), 0.2);
    assertEquals(2.0, calculateSpeed.getRight(), 0.2);
  }

  @Test
  public void puckFrontLeft() {
    Speed calculateSpeed = pdc.calculateSpeed(new double[] {0, 1, 0, 0, 0, 0});
    assertEquals(1.5, calculateSpeed.getLeft(), 0.2);
    assertEquals(2.0, calculateSpeed.getRight(), 0.2);
  }

  @Test
  public void puckLeft() {
    Speed calculateSpeed = pdc.calculateSpeed(new double[] {1, 0, 0, 0, 0, 0});
    assertEquals(0.5, calculateSpeed.getLeft(), 0.2);
    assertEquals(2.0, calculateSpeed.getRight(), 0.2);
  }

  @Test
  public void puckFrontRight() {
    Speed calculateSpeed = pdc.calculateSpeed(new double[] {0, 0, 0, 0, 1, 0});
    assertEquals(1.5, calculateSpeed.getRight(), 0.2);
    assertEquals(2.0, calculateSpeed.getLeft(), 0.2);
  }

  @Test
  public void puckRight() {
    Speed calculateSpeed = pdc.calculateSpeed(new double[] {0, 0, 0, 0, 0, 1});
    assertEquals(0.5, calculateSpeed.getRight(), 0.2);
    assertEquals(2.0, calculateSpeed.getLeft(), 0.2);
  }
}
