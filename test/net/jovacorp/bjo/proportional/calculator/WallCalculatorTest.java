package net.jovacorp.bjo.proportional.calculator;

import at.fhv.dgr1992.differentialWheels.Speed;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * We used the front and frontSide seonsors
 * {frontLeftSide, frontLeft, frontRight, frontRightSide}
 * the side sensors signals have to be inverted
 */
class WallCalculatorTest {
    private final WallCalculator wc = new WallCalculator();

    @Test
    public void wallLeftTest() {
        Speed calculateSpeed = wc.calculateSpeed(new double[]{0, 0, 0, 0});
        assertEquals(2.0, calculateSpeed.getLeft(), 0.2);
        assertEquals(2.0, calculateSpeed.getRight(), 0.2);
    }

    @Test
    public void wallLeftAndFrontTest() {
        Speed calculateSpeed = wc.calculateSpeed(new double[]{0, 1, 0, 0});
        assertEquals(2.0, calculateSpeed.getLeft(), 0.2);
        assertEquals(0.0, calculateSpeed.getRight(), 0.2);
    }

    @Test
    public void noWallLeftTest() {
        Speed calculateSpeed = wc.calculateSpeed(new double[]{1, 0, 0, 0});
        assertEquals(1.0, calculateSpeed.getLeft(), 0.2);
        assertEquals(2.0, calculateSpeed.getRight(), 0.2);
    }

}