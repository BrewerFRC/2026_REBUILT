package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Shared field-coordinate math for commands that point at or measure distance from the HUB.
 *
 * <p>WPILib poses use the blue-alliance-origin coordinate frame. Both odometry and
 * {@link HubPositionInfo} use that same frame, so no extra alliance flip belongs in this class.
 */
public final class HubAim {
  private HubAim() {}

  /** Returns the field-relative direction from {@code robotPosition} toward the current HUB. */
  public static Rotation2d headingToHub(Translation2d robotPosition) {
    return headingBetween(robotPosition, HubPositionInfo.hubPosition());
  }

  /** Returns the straight-line distance in meters from {@code robotPosition} to the current HUB. */
  public static double distanceToHub(Translation2d robotPosition) {
    return distanceBetween(robotPosition, HubPositionInfo.hubPosition());
  }

  /**
   * Returns the field-relative direction from one point to another. This pure method is used by
   * tests without needing a Driver Station alliance.
   */
  public static Rotation2d headingBetween(Translation2d from, Translation2d to) {
    return to.minus(from).getAngle();
  }

  /** Returns the straight-line distance in meters between two field positions. */
  public static double distanceBetween(Translation2d first, Translation2d second) {
    return first.getDistance(second);
  }
}
