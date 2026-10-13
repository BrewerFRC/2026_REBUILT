package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

/** Controls the ground-intake motor used to collect FUEL. */
public final class IntakeSubsystem extends SubsystemBase {
  private static final double INTAKE_POWER = 0.70;
  private static final double REVERSE_POWER = -0.40;

  private final SparkMax intakeMotor;
  private IntakeState state = IntakeState.OFF;

  /** The commanded state is published to help students diagnose controls and wiring. */
  public enum IntakeState {
    REVERSE,
    OFF,
    INTAKE
  }

  public IntakeSubsystem() {
    intakeMotor = new SparkMax(OperatorConstants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Intake/State", state.toString());
    SmartDashboard.putNumber("Intake/Motor Output", intakeMotor.get());
  }

  /** Starts intake at the tested collection speed. Commands call this every scheduler cycle. */
  public void intakeOn() {
    intakeMotor.set(INTAKE_POWER);
    state = IntakeState.INTAKE;
  }

  /** Stops the motor. This is the safe state whenever no intake command owns the subsystem. */
  public void intakeOff() {
    intakeMotor.stopMotor();
    state = IntakeState.OFF;
  }

  /** Runs the intake backwards to clear a jam or return FUEL to the field. */
  public void intakeReverse() {
    intakeMotor.set(REVERSE_POWER);
    state = IntakeState.REVERSE;
  }
}
