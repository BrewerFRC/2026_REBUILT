package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HubAim;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Faces the HUB and drives until the robot reaches the calibrated shooting distance.
 *
 * <p>This is a driver-assist command, not a complete autonomous path. The requested speed is
 * deliberately limited and must be field-tested before match use.
 */
public final class DriveToHubDistanceCommand extends Command {
  private static final double TARGET_DISTANCE_METRES = 2.6;
  private static final double DISTANCE_TOLERANCE_METRES = 0.1;
  private static final double MIN_APPROACH_SPEED_METRES_PER_SECOND = 0.6;
  private static final double DISTANCE_TO_SPEED_GAIN = 2.5;
  private static final double ALIGNMENT_TOLERANCE_RADIANS = Degrees.of(5).in(Radians);

  private final CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.FieldCentricFacingAngle aimRequest =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1)
          .withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective)
          .withHeadingPID(8, 0, 0);

  /** Creates the driver-assist command for the supplied drivetrain. */
  public DriveToHubDistanceCommand(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  /** Recomputes the HUB distance so a moving robot slows when it reaches the target range. */
  @Override
  public void execute() {
    double distanceErrorMetres = HubAim.distanceToHub(drivetrain.getState().Pose.getTranslation()) - TARGET_DISTANCE_METRES;
    boolean distanceReady = distanceErrorMetres <= DISTANCE_TOLERANCE_METRES;
    double velocityXMetresPerSecond =
        distanceReady
            ? 0.0
            : Math.max(MIN_APPROACH_SPEED_METRES_PER_SECOND, DISTANCE_TO_SPEED_GAIN * distanceErrorMetres);

    SmartDashboard.putNumber("Hub Approach Distance Error (m)", distanceErrorMetres);
    SmartDashboard.putBoolean("Hub Approach Distance Ready", distanceReady);

    drivetrain.setControl(
        aimRequest
            .withVelocityX(velocityXMetresPerSecond)
            .withVelocityY(0.0)
            .withTargetDirection(HubAim.headingToHub(drivetrain.getState().Pose.getTranslation())));
  }

  /** Driver-assist stays active while its binding is held. */
  @Override
  public boolean isFinished() {
    return false;
  }

  /** Returns true when the robot points within five degrees of the HUB. */
  public boolean isRobotAligned() {
    Rotation2d targetHeading = aimRequest.TargetDirection;
    Rotation2d currentHeading =
        drivetrain.getState().Pose.getRotation().rotateBy(drivetrain.getOperatorForwardDirection());
    return MathUtil.isNear(
        MathUtil.angleModulus(targetHeading.getRadians()),
        MathUtil.angleModulus(currentHeading.getRadians()),
        ALIGNMENT_TOLERANCE_RADIANS,
        -Math.PI,
        Math.PI);
  }
}
