package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HubAim;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;

/** Lets the driver translate while the drivetrain continuously faces the HUB. */
public final class ManualHubAimDriveCommand extends Command {
  private static final double JOYSTICK_DEADBAND = 0.10;
  private static final double DRIVE_INPUT_RATE_PER_SECOND = 1.0;
  private static final double MAX_SPEED_METRES_PER_SECOND =
      0.7 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private static final double MAX_ANGULAR_RATE_RADIANS_PER_SECOND =
      RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private final CommandSwerveDrivetrain drivetrain;
  private final DoubleSupplier forwardInput;
  private final DoubleSupplier leftInput;
  private final SlewRateLimiter forwardLimiter = new SlewRateLimiter(DRIVE_INPUT_RATE_PER_SECOND);
  private final SlewRateLimiter leftLimiter = new SlewRateLimiter(DRIVE_INPUT_RATE_PER_SECOND);
  private final SwerveRequest.FieldCentricFacingAngle aimRequest =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDeadband(MAX_SPEED_METRES_PER_SECOND * JOYSTICK_DEADBAND)
          .withRotationalDeadband(MAX_ANGULAR_RATE_RADIANS_PER_SECOND * JOYSTICK_DEADBAND)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective)
          .withHeadingPID(8, 0, 0);

  public ManualHubAimDriveCommand(
      CommandSwerveDrivetrain drivetrain, DoubleSupplier forwardInput, DoubleSupplier leftInput) {
    this.drivetrain = drivetrain;
    this.forwardInput = forwardInput;
    this.leftInput = leftInput;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    forwardLimiter.reset(0.0);
    leftLimiter.reset(0.0);
  }

  @Override
  public void execute() {
    double forward = forwardLimiter.calculate(MathUtil.applyDeadband(forwardInput.getAsDouble(), JOYSTICK_DEADBAND));
    double left = leftLimiter.calculate(MathUtil.applyDeadband(leftInput.getAsDouble(), JOYSTICK_DEADBAND));
    drivetrain.setControl(
        aimRequest
            .withVelocityX(forward * MAX_SPEED_METRES_PER_SECOND)
            .withVelocityY(left * MAX_SPEED_METRES_PER_SECOND)
            .withTargetDirection(HubAim.headingToHub(drivetrain.getState().Pose.getTranslation())));
  }

  @Override
  public void end(boolean interrupted) {
    // Facing-angle requests own rotation, so zero only translation before the default drive resumes.
    drivetrain.setControl(aimRequest.withVelocityX(0).withVelocityY(0));
  }
}
