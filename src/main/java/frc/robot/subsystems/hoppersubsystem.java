// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix6.configs.PWM2Configs;
//import com.revrobotics.PersistMode;
//import com.revrobotics.ResetMode;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
//import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.SparkLowLevel.MotorType;

// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;

//import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.PWM;

public class hoppersubsystem extends SubsystemBase {

    //public TalonFX hoppersubsystemMotor;
    public PWMMotorController hoppersubsystemMotor;
    //private TalonFXConfiguration configs = new TalonFXConfiguration();
    //private SparkMax hoppersubsystemMotor;

    double targetpower = - 0.75;  //was -0.4
    double startpower = 0;
    double power = startpower;

    public enum hoppersubsystemStates{
        HOPPER_REVERSE,    // The hoppersubsystem is reversed
        HOPPER_OFF,    // The hoppersubsystem is off
        HOPPER_ON;     // hoppersubsystem is on
    }
    public hoppersubsystemStates hoppersubsystemState = hoppersubsystemStates.HOPPER_OFF; //Default state
    
    /** Creates a new hoppersubsystem. */
    public hoppersubsystem() {
        //hoppersubsystemMotor = new TalonFX(OperatorConstants.hopperMotorID,OperatorConstants.canbusname);
        hoppersubsystemMotor = new Spark(OperatorConstants.hopperMotorID);
        hoppersubsystemMotor.setInverted(true);
        //hoppersubsystemMotor = new SparkMax(OperatorConstants.hopperMotorID, MotorType.kBrushed);
        
        // hoppersubsystemMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putString("hoppersubsystem State",hoppersubsystemState.toString());
        SmartDashboard.putNumber("hoppersubsystem Motor Speed",hoppersubsystemMotor.get());
    }

    public void init() {
        hoppersubsystemState = hoppersubsystemStates.HOPPER_OFF;
    }

    public void hopperOn() { //double targetPower
        // if (power > targetPower) {
        //     power =- 0.25;
        // }
        if (Math.abs(power) < Math.abs(targetpower)){
            if (targetpower < 0){
                power = power - 0.005;
            } else {
                power = power + 0.005;
            }
        }
        hoppersubsystemMotor.set(power);
        //System.out.println("Hopper On");
        hoppersubsystemState = hoppersubsystemStates.HOPPER_ON;
    }

    public void hopperOff() {
        hoppersubsystemMotor.set(0);
        //power = -0.3;
        power = 0;
        //System.out.println("Hopper Off");
        hoppersubsystemState = hoppersubsystemStates.HOPPER_OFF;
    }

    public void hopperReverse() {
        hoppersubsystemMotor.set(0.55);
        //System.out.println("Hopper Reversed");
        hoppersubsystemState = hoppersubsystemStates.HOPPER_REVERSE;
    }
}