package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

/** Tests pure HUB geometry without requiring roboRIO hardware or Driver Station data. */
class HubAimTest {
  private static final double EPSILON = 1e-9;

  @Test
  void distanceBetweenUsesPythagoreanDistance() {
    assertEquals(
        5.0,
        HubAim.distanceBetween(new Translation2d(1.0, 2.0), new Translation2d(4.0, 6.0)),
        EPSILON);
  }

  @Test
  void headingBetweenPointsTowardTargetInFieldCoordinates() {
    assertEquals(
        Math.PI / 2.0,
        HubAim.headingBetween(new Translation2d(2.0, 1.0), new Translation2d(2.0, 4.0)).getRadians(),
        EPSILON);
    assertEquals(
        0.0,
        HubAim.headingBetween(new Translation2d(2.0, 1.0), new Translation2d(5.0, 1.0)).getRadians(),
        EPSILON);
  }
}
