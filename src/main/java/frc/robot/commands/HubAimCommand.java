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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HubAim;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Turns the robot toward its alliance HUB while allowing a caller to specify field-relative
 * translation speeds.
 *
 * <p>This command requires the drivetrain, so it intentionally interrupts the normal driver
 * drive command while it is scheduled. Releasing its trigger returns control to the default drive
 * command.
 */
public final class HubAimCommand extends Command {
  private static final double ALIGNMENT_TOLERANCE_RADIANS = Degrees.of(3).in(Radians);

  private final CommandSwerveDrivetrain drivetrain;
  private final double velocityXMetresPerSecond;
  private final double velocityYMetresPerSecond;
  private final SwerveRequest.FieldCentricFacingAngle aimRequest =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1)
          .withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective)
          .withHeadingPID(8, 0, 0);

  /** Creates an aim command with the requested field-relative translation speeds in m/s. */
  public HubAimCommand(
      CommandSwerveDrivetrain drivetrain,
      double velocityXMetresPerSecond,
      double velocityYMetresPerSecond) {
    this.drivetrain = drivetrain;
    this.velocityXMetresPerSecond = velocityXMetresPerSecond;
    this.velocityYMetresPerSecond = velocityYMetresPerSecond;
    addRequirements(drivetrain);
  }

  /** Updates the requested heading every scheduler cycle as odometry changes. */
  @Override
  public void execute() {
    drivetrain.setControl(
        aimRequest
            .withVelocityX(velocityXMetresPerSecond)
            .withVelocityY(velocityYMetresPerSecond)
            .withTargetDirection(HubAim.headingToHub(drivetrain.getState().Pose.getTranslation())));
  }

  /** This command continues until its button binding or parent command cancels it. */
  @Override
  public boolean isFinished() {
    return false;
  }

  /** Returns true when the robot heading is within three degrees of the requested HUB direction. */
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
