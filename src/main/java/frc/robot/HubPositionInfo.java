package frc.robot;
import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Field-relative info about the scoring hub's position, mirrored across the
 * two alliance-specific sides of the field.
 */
public final class HubPositionInfo {
  // Manual §5.2 and §5.4. Keep these named so they can be replaced with the
  // official field-drawing values if FIRST revises the field between events.
  private static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.2);
  private static final double FIELD_WIDTH_METERS = Units.inchesToMeters(317.7);
  private static final double HUB_DISTANCE_FROM_ALLIANCE_WALL_METERS = Units.inchesToMeters(158.6);

  private HubPositionInfo() {}

  /** Returns the hub's fixed field position for whichever alliance we're on. */
  public static Translation2d hubPosition() {
    final Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
      return new Translation2d(HUB_DISTANCE_FROM_ALLIANCE_WALL_METERS, FIELD_WIDTH_METERS / 2.0);
    }
    return new Translation2d(
        FIELD_LENGTH_METERS - HUB_DISTANCE_FROM_ALLIANCE_WALL_METERS, FIELD_WIDTH_METERS / 2.0);
  }
}
