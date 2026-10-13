// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;

//import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.DigitalInput;

/** Controls the single TalonFX motor that raises and lowers the arm. */
public class ArmSubsystem extends SubsystemBase {

  public TalonFX ArmSubsystemMotor;
  //private TalonFXConfiguration configs = new TalonFXConfiguration();

  // private DigitalInput lowerLimit02 = new DigitalInput(Constants.OperatorConstants.lowerlimitID02);
  // private DigitalInput lowerLimit01 = new DigitalInput(Constants.OperatorConstants.lowerlimitID01);
  // private DigitalInput upperLimit = new DigitalInput(Constants.OperatorConstants.upperlimitID);

  public enum ArmSubsystemStates{
        MOVING_UP,    // The ArmSubsystem is moving up
        MOVING_DOWN,    // The ArmSubsystem is moving down
        ARM_UP,       // ArmSubsystem is up
        ARM_DOWN;     // ArmSubsystem is down
  }
  public ArmSubsystemStates ArmSubsystemState = ArmSubsystemStates.ARM_UP; //Default state

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    ArmSubsystemMotor = new TalonFX(OperatorConstants.ARM_MOTOR_CAN_ID,OperatorConstants.CANIVORE_BUS);
    final TalonFXConfiguration armMotorConfig = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(50) // not present before
            .withStatorCurrentLimitEnable(true) //not present before
            .withSupplyCurrentLimit(50) //not present before
            .withStatorCurrentLimitEnable(true))    //not present before
        .withClosedLoopRamps(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.01));

    ArmSubsystemMotor.getConfigurator().apply(armMotorConfig);
    ArmSubsystemMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("ArmSubsystem State",ArmSubsystemState.toString());
    SmartDashboard.putNumber("ArmSubsystem Motor Speed",ArmSubsystemMotor.get());
    SmartDashboard.putNumber("Arm Encoder Value", ArmSubsystemMotor.getPosition().getValueAsDouble());
  }

  public void init(){
      //ArmSubsystemMotor.setInverted(true);
      ArmSubsystemMotor.setPosition(0);
      ArmSubsystemUP();
    /*configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 50.0;
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    ArmSubsystemMotor.getConfigurator().apply(configs);*/

  }

  public boolean ArmlowerLimit(){
    if (Math.abs(Math.abs(ArmSubsystemMotor.getPosition().getValueAsDouble()) - 17.4) < 0.5) {  //lowerLimit01.get() || lowerLimit02.get() ||
      ArmSubsystemSTOP();
      return true;
    } else {
      return false;
    }
  }

  public boolean ArmUpperLimit(){  //limit of somesort
    if (Math.abs(Math.abs(ArmSubsystemMotor.getPosition().getValueAsDouble())) < 0.5){     //!upperLimit.get() ||
        ArmSubsystemSTOP();
        return true;
      } else {
        return false;
      }
    //ArmSubsystemMotor.getFault_ForwardSoftLimit().getValue()
  }

  public void ArmSubsystemUP(){
    // if (ArmUpperLimit()){
    //   ArmSubsystemSTOP();
    //   ArmSubsystemState = ArmSubsystemState.ARM_UP;
    // } else {
      ArmSubsystemMotor.set(-0.3);
      ArmSubsystemState = ArmSubsystemState.MOVING_UP;
    // }

    //System.out.println("Arm Up");
  }

  public void ArmSubsystemDown(){
    //System.out.println("Arm Down");
    // if (ArmlowerLimit()){
    //   ArmSubsystemSTOP();
    //   ArmSubsystemState = ArmSubsystemState.ARM_DOWN;
    // } else {
      ArmSubsystemMotor.set(0.3);
      ArmSubsystemState = ArmSubsystemStates.MOVING_DOWN;
    //}
  }

  public void ArmSubsystemSTOP(){
    ArmSubsystemMotor.set(0);
    //System.out.println("Arm Stopped");
  }
}