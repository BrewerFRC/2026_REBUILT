package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HubAim;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Lets the driver translate with the joysticks while the drivetrain automatically faces the HUB.
 *
 * <p>Rotation input is intentionally not accepted: the command owns drivetrain rotation so it can
 * keep the shooter pointed at the target. Translation input is ramped to prevent abrupt commands.
 */
public final class ManualHubAimDriveCommand extends Command {
  private static final double MAX_SPEED_METRES_PER_SECOND =
      0.7 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private static final double MAX_ANGULAR_RATE_RADIANS_PER_SECOND =
      RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private static final double DRIVE_RAMP_PER_CYCLE = 0.02;
  private static final int STOP_AFTER_IDLE_CYCLES = 2;

  private final CommandSwerveDrivetrain drivetrain;
  private final DoubleSupplier forwardInput;
  private final DoubleSupplier leftInput;
  private final SwerveRequest.FieldCentricFacingAngle aimRequest =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDeadband(MAX_SPEED_METRES_PER_SECOND * 0.1)
          .withRotationalDeadband(MAX_ANGULAR_RATE_RADIANS_PER_SECOND * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective)
          .withHeadingPID(8, 0, 0);

  private double currentForwardInput;
  private double currentLeftInput;
  private int idleCycles;

  /** Creates a HUB-facing driver-assist command from normalized joystick suppliers. */
  public ManualHubAimDriveCommand(
      CommandSwerveDrivetrain drivetrain, DoubleSupplier forwardInput, DoubleSupplier leftInput) {
    this.drivetrain = drivetrain;
    this.forwardInput = forwardInput;
    this.leftInput = leftInput;
    addRequirements(drivetrain);
  }

  /** Starts each button press from a stopped input state. */
  @Override
  public void initialize() {
    currentForwardInput = 0.0;
    currentLeftInput = 0.0;
    idleCycles = 0;
  }

  /** Ramps translation and continuously updates the field-relative HUB heading. */
  @Override
  public void execute() {
    double requestedForward = forwardInput.getAsDouble();
    double requestedLeft = leftInput.getAsDouble();
    idleCycles = requestedForward == 0.0 && requestedLeft == 0.0 ? idleCycles + 1 : 0;

    if (idleCycles > STOP_AFTER_IDLE_CYCLES) {
      currentForwardInput = 0.0;
      currentLeftInput = 0.0;
    } else {
      currentForwardInput = ramp(currentForwardInput, requestedForward, DRIVE_RAMP_PER_CYCLE);
      currentLeftInput = ramp(currentLeftInput, requestedLeft, DRIVE_RAMP_PER_CYCLE);
    }

    drivetrain.setControl(
        aimRequest
            .withVelocityX(currentForwardInput * MAX_SPEED_METRES_PER_SECOND)
            .withVelocityY(currentLeftInput * MAX_SPEED_METRES_PER_SECOND)
            .withTargetDirection(HubAim.headingToHub(drivetrain.getState().Pose.getTranslation())));
  }

  /** Driver-assist stays active while its binding is held. */
  @Override
  public boolean isFinished() {
    return false;
  }

  /** Moves a value toward its target by no more than {@code rampPerCycle}. */
  public static double ramp(double currentValue, double targetValue, double rampPerCycle) {
    if (currentValue < targetValue) {
      return Math.min(currentValue + rampPerCycle, targetValue);
    }
    return Math.max(currentValue - rampPerCycle, targetValue);
  }
}
