package util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.util.RotationUtil;
import org.junit.jupiter.api.Test;

public class RotationUtilTest {
  public static final double EPSILON = 1e-6;

  @Test
  void deltaAngleDegreesTest() {
    assertEquals(RotationUtil.deltaAngleDegrees(-30, 20), 50.0, EPSILON);
    assertEquals(RotationUtil.deltaAngleDegrees(20, -30), -50.0, EPSILON);
    assertEquals(RotationUtil.deltaAngleDegrees(330, 20), 50.0, EPSILON);
    assertEquals(RotationUtil.deltaAngleDegrees(20, 330), -50.0, EPSILON);
    assertEquals(RotationUtil.deltaAngleDegrees(30, -330), 0.0, EPSILON);
    assertEquals(RotationUtil.deltaAngleDegrees(30, 209), 179.0, EPSILON);
    assertEquals(RotationUtil.deltaAngleDegrees(30, 210), 180.0, EPSILON);
    assertEquals(RotationUtil.deltaAngleDegrees(30, 211), -179.0, EPSILON);
  }
}
