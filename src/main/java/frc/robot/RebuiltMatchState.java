package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Interprets the 2026 REBUILT match schedule and FMS game-data message.
 *
 * <p>The FMS sends {@code R} or {@code B} shortly after AUTO. That character identifies the
 * alliance whose HUB is inactive first. The match time reported by the Driver Station is
 * approximate, so this class is intentionally a scoring aid, not a replacement for watching the
 * field lights.
 */
public final class RebuiltMatchState {
  private static final double TRANSITION_END_SECONDS = 130.0;
  private static final double SHIFT_1_END_SECONDS = 105.0;
  private static final double SHIFT_2_END_SECONDS = 80.0;
  private static final double SHIFT_3_END_SECONDS = 55.0;
  private static final double SHIFT_4_END_SECONDS = 30.0;

  /** REBUILT's scoring windows during TELEOP. */
  public enum HubPhase {
    DISABLED,
    AUTONOMOUS,
    TRANSITION,
    SHIFT_1,
    SHIFT_2,
    SHIFT_3,
    SHIFT_4,
    ENDGAME
  }

  private RebuiltMatchState() {}

  /** Returns whether this robot's HUB should award points right now. */
  public static boolean isHubActive() {
    return isHubActive(
        DriverStation.getAlliance(),
        DriverStation.isAutonomousEnabled(),
        DriverStation.isTeleopEnabled(),
        DriverStation.getMatchTime(),
        DriverStation.getGameSpecificMessage());
  }

  /**
   * Pure form of {@link #isHubActive()} for simulation and future unit tests.
   *
   * <p>Before FMS data arrives (or if it is malformed), this returns active during TELEOP. This
   * matches WPILib's 2026 guidance: data is normally absent only in the active transition period,
   * and a missing message must not make the robot unusable.
   */
  public static boolean isHubActive(
      Optional<Alliance> alliance,
      boolean autonomousEnabled,
      boolean teleopEnabled,
      double matchTimeSeconds,
      String gameData) {
    if (alliance.isEmpty()) {
      return false;
    }
    if (autonomousEnabled) {
      return true;
    }
    if (!teleopEnabled) {
      return false;
    }

    if (gameData == null || gameData.isEmpty()) {
      return true;
    }

    final boolean redInactiveFirst;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        return true;
      }
    }

    // SHIFT 1 is active for Blue when Red was inactive first, and vice versa.
    final boolean shiftOneActive =
        alliance.get() == Alliance.Red ? !redInactiveFirst : redInactiveFirst;

    if (matchTimeSeconds > TRANSITION_END_SECONDS) {
      return true;
    } else if (matchTimeSeconds > SHIFT_1_END_SECONDS) {
      return shiftOneActive;
    } else if (matchTimeSeconds > SHIFT_2_END_SECONDS) {
      return !shiftOneActive;
    } else if (matchTimeSeconds > SHIFT_3_END_SECONDS) {
      return shiftOneActive;
    } else if (matchTimeSeconds > SHIFT_4_END_SECONDS) {
      return !shiftOneActive;
    }
    return true;
  }

  /** Returns the current REBUILT scoring window for operator feedback. */
  public static HubPhase getHubPhase() {
    if (DriverStation.isAutonomousEnabled()) {
      return HubPhase.AUTONOMOUS;
    }
    if (!DriverStation.isTeleopEnabled()) {
      return HubPhase.DISABLED;
    }

    final double matchTimeSeconds = DriverStation.getMatchTime();
    if (matchTimeSeconds > TRANSITION_END_SECONDS) {
      return HubPhase.TRANSITION;
    } else if (matchTimeSeconds > SHIFT_1_END_SECONDS) {
      return HubPhase.SHIFT_1;
    } else if (matchTimeSeconds > SHIFT_2_END_SECONDS) {
      return HubPhase.SHIFT_2;
    } else if (matchTimeSeconds > SHIFT_3_END_SECONDS) {
      return HubPhase.SHIFT_3;
    } else if (matchTimeSeconds > SHIFT_4_END_SECONDS) {
      return HubPhase.SHIFT_4;
    }
    return HubPhase.ENDGAME;
  }
}
