package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.junit.jupiter.api.Test;

class RebuiltMatchStateTest {
  @Test
  void hubIsActiveDuringAutonomousAndTransition() {
    assertTrue(RebuiltMatchState.isHubActive(Optional.of(Alliance.Red), true, false, 0.0, ""));
    assertTrue(RebuiltMatchState.isHubActive(Optional.of(Alliance.Red), false, true, 140.0, "R"));
  }

  @Test
  void redIsInactiveInOddShiftsWhenRedIsInGameData() {
    assertFalse(RebuiltMatchState.isHubActive(Optional.of(Alliance.Red), false, true, 120.0, "R"));
    assertTrue(RebuiltMatchState.isHubActive(Optional.of(Alliance.Red), false, true, 100.0, "R"));
    assertFalse(RebuiltMatchState.isHubActive(Optional.of(Alliance.Red), false, true, 70.0, "R"));
    assertTrue(RebuiltMatchState.isHubActive(Optional.of(Alliance.Red), false, true, 40.0, "R"));
  }

  @Test
  void blueMirrorsRedShiftStatusAndEndgameIsAlwaysActive() {
    assertTrue(RebuiltMatchState.isHubActive(Optional.of(Alliance.Blue), false, true, 120.0, "R"));
    assertFalse(RebuiltMatchState.isHubActive(Optional.of(Alliance.Blue), false, true, 100.0, "R"));
    assertTrue(RebuiltMatchState.isHubActive(Optional.of(Alliance.Blue), false, true, 20.0, "R"));
  }

  @Test
  void missingAllianceIsSafeAndMissingGameDataDoesNotBlockTransition() {
    assertFalse(RebuiltMatchState.isHubActive(Optional.empty(), false, true, 120.0, "R"));
    assertTrue(RebuiltMatchState.isHubActive(Optional.of(Alliance.Blue), false, true, 120.0, ""));
  }
}
