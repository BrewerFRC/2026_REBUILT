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

/** Controls the hopper motor that moves game pieces up toward the feeder. */
public class HopperSubsystem extends SubsystemBase {

    //public TalonFX HopperSubsystemMotor;
    public PWMMotorController HopperSubsystemMotor;
    //private TalonFXConfiguration configs = new TalonFXConfiguration();
    //private SparkMax HopperSubsystemMotor;

    double targetpower = - 0.75;  //was -0.4
    double startpower = 0;
    double power = startpower;

    public enum HopperSubsystemStates{
        HOPPER_REVERSE,    // The HopperSubsystem is reversed
        HOPPER_OFF,    // The HopperSubsystem is off
        HOPPER_ON;     // HopperSubsystem is on
    }
    public HopperSubsystemStates HopperSubsystemState = HopperSubsystemStates.HOPPER_OFF; //Default state

    /** Creates a new HopperSubsystem. */
    public HopperSubsystem() {
        //HopperSubsystemMotor = new TalonFX(OperatorConstants.HOPPER_MOTOR_PWM_CHANNEL,OperatorConstants.CANIVORE_BUS);
        HopperSubsystemMotor = new Spark(OperatorConstants.HOPPER_MOTOR_PWM_CHANNEL);
        HopperSubsystemMotor.setInverted(true);
        //HopperSubsystemMotor = new SparkMax(OperatorConstants.HOPPER_MOTOR_PWM_CHANNEL, MotorType.kBrushed);

        // HopperSubsystemMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putString("HopperSubsystem State",HopperSubsystemState.toString());
        SmartDashboard.putNumber("HopperSubsystem Motor Speed",HopperSubsystemMotor.get());
    }

    public void init() {
        HopperSubsystemState = HopperSubsystemStates.HOPPER_OFF;
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
        HopperSubsystemMotor.set(power);
        //System.out.println("Hopper On");
        HopperSubsystemState = HopperSubsystemStates.HOPPER_ON;
    }

    public void hopperOff() {
        HopperSubsystemMotor.set(0);
        //power = -0.3;
        power = 0;
        //System.out.println("Hopper Off");
        HopperSubsystemState = HopperSubsystemStates.HOPPER_OFF;
    }

    public void hopperReverse() {
        HopperSubsystemMotor.set(0.55);
        //System.out.println("Hopper Reversed");
        HopperSubsystemState = HopperSubsystemStates.HOPPER_REVERSE;
    }
}