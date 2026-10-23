package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

/** Controls the feeder motor that moves FUEL from the hopper into the shooter. */
public class FeederSubsystem extends SubsystemBase {
  private static final double FEEDER_RPS = 75;
  private static final double PREPARE_RPS = -25;
  private static final double REVERSE_RPS = -50;
  private static final double PREPARE_SECONDS = 0.50;
  private final TalonFX feederMotor;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final Timer feedPreparationTimer = new Timer();

  public enum FeederSubsystemStates { FEEDER_REVERSE, FEEDER_ON, FEEDER_OFF }
  public FeederSubsystemStates FeederSubsystemState = FeederSubsystemStates.FEEDER_OFF;

  public FeederSubsystem() {
    feederMotor = new TalonFX(OperatorConstants.FEEDER_MOTOR_CAN_ID, OperatorConstants.CANIVORE_BUS);
    TalonFXConfiguration feederMotorConfig = new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.CounterClockwise_Positive))
        .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(50)
            .withStatorCurrentLimitEnable(true).withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true))
        .withSlot0(new Slot0Configs().withKP(0.5).withKS(0.28).withKV(0.1))
        .withClosedLoopRamps(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.1));
    SmartDashboard.putBoolean(
        "Feeder/Configuration Applied", feederMotor.getConfigurator().apply(feederMotorConfig).isOK());
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("FeederSubsystem State", FeederSubsystemState.toString());
    SmartDashboard.putNumber("Feeder/Velocity RPS", feederMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Feeder/Applied Volts", feederMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Feeder/Supply Current Amps", feederMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Feeder/Stator Current Amps", feederMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Feeder/Temperature C", feederMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Feeder/Fault Bits", feederMotor.getFaultField().getValueAsDouble());
  }

  /** Starts the one-time, half-second reverse preparation for a new feed request. */
  public void beginFeeding() {
    feedPreparationTimer.restart();
    FeederSubsystemState = FeederSubsystemStates.FEEDER_OFF;
  }

  /** Runs preparation briefly, then forward feed; duration is independent of scheduler frequency. */
  public void feederOn() {
    double targetRps = feedPreparationTimer.hasElapsed(PREPARE_SECONDS) ? FEEDER_RPS : PREPARE_RPS;
    feederMotor.setControl(velocityRequest.withVelocity(targetRps));
    FeederSubsystemState = FeederSubsystemStates.FEEDER_ON;
  }

  public void feederOff() {
    feedPreparationTimer.stop();
    feedPreparationTimer.reset();
    feederMotor.setControl(velocityRequest.withVelocity(0));
    FeederSubsystemState = FeederSubsystemStates.FEEDER_OFF;
  }

  public void feederReverse() {
    feedPreparationTimer.stop();
    feederMotor.setControl(velocityRequest.withVelocity(REVERSE_RPS));
    FeederSubsystemState = FeederSubsystemStates.FEEDER_REVERSE;
  }
}
