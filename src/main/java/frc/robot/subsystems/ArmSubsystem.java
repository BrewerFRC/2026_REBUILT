package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

/** Controls the arm using its integrated motor encoder as the sole position reference. */
public class ArmSubsystem extends SubsystemBase {
  /** The known, fully retracted position at robot power-on. Verify this before enabling. */
  private static final double STOWED_POSITION_ROTATIONS = 0.0;
  private static final double EXTENDED_POSITION_ROTATIONS = 17.4;
  private static final double POSITION_TOLERANCE_ROTATIONS = 0.5;
  // 30% duty cycle at a nominal 12 V battery, now held constant as battery voltage changes.
  private static final double MANUAL_ARM_VOLTAGE = 3.6;
  private static final double OPEN_LOOP_RAMP_SECONDS = 0.15;

  public final TalonFX ArmSubsystemMotor;
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public enum ArmSubsystemStates { MOVING_UP, MOVING_DOWN, ARM_UP, ARM_DOWN }

  public ArmSubsystemStates ArmSubsystemState = ArmSubsystemStates.ARM_UP;

  /** Creates the motor configuration and establishes the encoder reference exactly once. */
  public ArmSubsystem() {
    ArmSubsystemMotor =
        new TalonFX(OperatorConstants.ARM_MOTOR_CAN_ID, OperatorConstants.CANIVORE_BUS);
    TalonFXConfiguration armMotorConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(50)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(50)
                    .withSupplyCurrentLimitEnable(true))
            .withOpenLoopRamps(
                new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(OPEN_LOOP_RAMP_SECONDS));

    SmartDashboard.putBoolean(
        "Arm/Configuration Applied", ArmSubsystemMotor.getConfigurator().apply(armMotorConfig).isOK());
    // There are no limit switches. This is valid only when the robot begins physically stowed.
    ArmSubsystemMotor.setPosition(STOWED_POSITION_ROTATIONS);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("ArmSubsystem State", ArmSubsystemState.toString());
    SmartDashboard.putNumber("Arm/Position Rotations", positionRotations());
    SmartDashboard.putNumber("Arm/Applied Volts", ArmSubsystemMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Arm/Supply Current Amps", ArmSubsystemMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Arm/Stator Current Amps", ArmSubsystemMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Arm/Temperature C", ArmSubsystemMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Arm/Fault Bits", ArmSubsystemMotor.getFaultField().getValueAsDouble());
  }

  /** Returns whether the arm is at its extended travel boundary. */
  public boolean ArmlowerLimit() {
    return positionRotations() >= EXTENDED_POSITION_ROTATIONS - POSITION_TOLERANCE_ROTATIONS;
  }

  /** Returns whether the arm is at its stowed travel boundary. */
  public boolean ArmUpperLimit() {
    return positionRotations() <= STOWED_POSITION_ROTATIONS + POSITION_TOLERANCE_ROTATIONS;
  }

  /** Moves toward stow, but never commands farther past the stowed encoder boundary. */
  public void ArmSubsystemUP() {
    if (ArmUpperLimit()) {
      ArmSubsystemSTOP();
      ArmSubsystemState = ArmSubsystemStates.ARM_UP;
      return;
    }
    ArmSubsystemMotor.setControl(voltageRequest.withOutput(-MANUAL_ARM_VOLTAGE));
    ArmSubsystemState = ArmSubsystemStates.MOVING_UP;
  }

  /** Moves toward extension, but never commands farther past the extended encoder boundary. */
  public void ArmSubsystemDown() {
    if (ArmlowerLimit()) {
      ArmSubsystemSTOP();
      ArmSubsystemState = ArmSubsystemStates.ARM_DOWN;
      return;
    }
    ArmSubsystemMotor.setControl(voltageRequest.withOutput(MANUAL_ARM_VOLTAGE));
    ArmSubsystemState = ArmSubsystemStates.MOVING_DOWN;
  }

  /** Immediately removes the arm command; commands also call this from {@code end()}. */
  public void ArmSubsystemSTOP() {
    ArmSubsystemMotor.setControl(voltageRequest.withOutput(0));
  }

  private double positionRotations() {
    return ArmSubsystemMotor.getPosition().getValueAsDouble();
  }
}
