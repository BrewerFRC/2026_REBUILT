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
public class armsubsystem extends SubsystemBase {
  
  public TalonFX armsubsystemMotor;
  //private TalonFXConfiguration configs = new TalonFXConfiguration();

  // private DigitalInput lowerLimit02 = new DigitalInput(Constants.OperatorConstants.lowerlimitID02);
  // private DigitalInput lowerLimit01 = new DigitalInput(Constants.OperatorConstants.lowerlimitID01);
  // private DigitalInput upperLimit = new DigitalInput(Constants.OperatorConstants.upperlimitID);
  
  public enum armsubsystemStates{
        MOVING_UP,    // The armsubsystem is moving up
        MOVING_DOWN,    // The armsubsystem is moving down
        ARM_UP,       // armsubsystem is up
        ARM_DOWN;     // armsubsystem is down
  }
  public armsubsystemStates armsubsystemState = armsubsystemStates.ARM_UP; //Default state
  
  /** Creates a new armsubsystem. */
  public armsubsystem() {
    armsubsystemMotor = new TalonFX(OperatorConstants.armMotorID,OperatorConstants.canbusname);
    final TalonFXConfiguration armMotorConfig = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(50) // not present before
            .withStatorCurrentLimitEnable(true) //not present before
            .withSupplyCurrentLimit(50) //not present before
            .withStatorCurrentLimitEnable(true))    //not present before
        .withClosedLoopRamps(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.01));

    armsubsystemMotor.getConfigurator().apply(armMotorConfig);
    armsubsystemMotor.setPosition(0); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("armsubsystem State",armsubsystemState.toString());
    SmartDashboard.putNumber("armsubsystem Motor Speed",armsubsystemMotor.get());
    SmartDashboard.putNumber("Arm Encoder Value", armsubsystemMotor.getPosition().getValueAsDouble());
  }

  public void init(){
      //armsubsystemMotor.setInverted(true);
      armsubsystemMotor.setPosition(0);
      armsubsystemUP();
    /*configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 50.0;
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    armsubsystemMotor.getConfigurator().apply(configs);*/
    
  }
  
  public boolean ArmlowerLimit(){
    if (Math.abs(Math.abs(armsubsystemMotor.getPosition().getValueAsDouble()) - 17.4) < 0.5) {  //lowerLimit01.get() || lowerLimit02.get() || 
      armsubsystemSTOP();
      return true;
    } else {
      return false;
    }
  }

  public boolean ArmUpperLimit(){  //limit of somesort
    if (Math.abs(Math.abs(armsubsystemMotor.getPosition().getValueAsDouble())) < 0.5){     //!upperLimit.get() || 
        armsubsystemSTOP();
        return true;
      } else {
        return false;
      }
    //armsubsystemMotor.getFault_ForwardSoftLimit().getValue()
  }

  public void armsubsystemUP(){
    // if (ArmUpperLimit()){  
    //   armsubsystemSTOP();
    //   armsubsystemState = armsubsystemState.ARM_UP;
    // } else {
      armsubsystemMotor.set(-0.3);
      armsubsystemState = armsubsystemState.MOVING_UP;
    // }

    //System.out.println("Arm Up");
  }

  public void armsubsystemDown(){
    //System.out.println("Arm Down");
    // if (ArmlowerLimit()){
    //   armsubsystemSTOP();
    //   armsubsystemState = armsubsystemState.ARM_DOWN;
    // } else {
      armsubsystemMotor.set(0.3);
      armsubsystemState = armsubsystemStates.MOVING_DOWN;
    //}
  }

  public void armsubsystemSTOP(){
    armsubsystemMotor.set(0);
    //System.out.println("Arm Stopped");
  }
}