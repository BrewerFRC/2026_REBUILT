package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/** Verifies the small pure helper used to smooth driver commands. */
class DriveInputRampTest {
  private static final double EPSILON = 1e-9;

  @Test
  void rampMovesTowardTargetWithoutOvershooting() {
    assertEquals(0.2, ManualHubAimDriveCommand.ramp(0.0, 1.0, 0.2), EPSILON);
    assertEquals(-0.2, ManualHubAimDriveCommand.ramp(0.0, -1.0, 0.2), EPSILON);
    assertEquals(0.1, ManualHubAimDriveCommand.ramp(0.0, 0.1, 0.2), EPSILON);
  }
}
