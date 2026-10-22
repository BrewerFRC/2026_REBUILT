package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;

/** Default field-centric teleop drive with time-based joystick slew limiting. */
public class TeleopDriveCommand extends Command {
  private static final double JOYSTICK_DEADBAND = 0.10;
  private static final double DRIVE_INPUT_RATE_PER_SECOND = 1.0;
  private static final double TURN_INPUT_RATE_PER_SECOND = 2.0;
  public static final double MaxSpeed = 0.7 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private final CommandSwerveDrivetrain drive;
  private final DoubleSupplier getLeftY;
  private final DoubleSupplier getLeftX;
  private final DoubleSupplier getRightX;
  private final SlewRateLimiter forwardLimiter = new SlewRateLimiter(DRIVE_INPUT_RATE_PER_SECOND);
  private final SlewRateLimiter leftLimiter = new SlewRateLimiter(DRIVE_INPUT_RATE_PER_SECOND);
  private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(TURN_INPUT_RATE_PER_SECOND);
  private final SwerveRequest.FieldCentric fieldcentricrequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * JOYSTICK_DEADBAND)
          .withRotationalDeadband(MaxAngularRate * JOYSTICK_DEADBAND)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public TeleopDriveCommand(
      CommandSwerveDrivetrain commandSwerveDrivetrain,
      DoubleSupplier getLeftY,
      DoubleSupplier getLeftX,
      DoubleSupplier getRightX) {
    drive = commandSwerveDrivetrain;
    this.getLeftY = getLeftY;
    this.getLeftX = getLeftX;
    this.getRightX = getRightX;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    forwardLimiter.reset(0.0);
    leftLimiter.reset(0.0);
    rotationLimiter.reset(0.0);
  }

  @Override
  public void execute() {
    double forward = forwardLimiter.calculate(MathUtil.applyDeadband(getLeftY.getAsDouble(), JOYSTICK_DEADBAND));
    double left = leftLimiter.calculate(MathUtil.applyDeadband(getLeftX.getAsDouble(), JOYSTICK_DEADBAND));
    double rotation = rotationLimiter.calculate(MathUtil.applyDeadband(getRightX.getAsDouble(), JOYSTICK_DEADBAND));
    drive.setControl(
        fieldcentricrequest
            .withVelocityX(forward * MaxSpeed)
            .withVelocityY(left * MaxSpeed)
            .withRotationalRate(rotation * MaxAngularRate));
  }

  @Override
  public void end(boolean interrupted) {
    drive.setControl(fieldcentricrequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }
}
