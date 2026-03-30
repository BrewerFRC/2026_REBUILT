// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.PersistMode;
//import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

//import edu.wpi.first.wpilibj.AnalogInput;

public class intakesubsystem extends SubsystemBase {
  
    private SparkMax intakeSubsystemMotor;
    //private TalonFXConfiguration configs = new TalonFXConfiguration();
    SparkMaxConfig config2Config = new SparkMaxConfig();
    //config2Config.inverted(true);
    //intakeSubsystemMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    private double power = 0.7;  //0.7, 0.6, 0.5, 0.3

    public enum intakesubsystemStates{
        INTAKE_REVERSE,    // The intakesubsystem is reversed
        INTAKE_OFF,    // The intakesubsystem is off
        INTAKE_ON;     // intakesubsystem is on
    }
    public intakesubsystemStates intakesubsystemState = intakesubsystemStates.INTAKE_OFF; //Default state
    
    /** Creates a new intakesubsystem. */
    public intakesubsystem() {
        intakeSubsystemMotor = new SparkMax(OperatorConstants.intakeMotorID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        //config.smartCurrentLimit(50);
        //config.secondaryCurrentLimit(50);

        //intakeSubsystemMotor.configure(config, null, PersistMode.kPersistParameters);

        //config.inverted(true);
        //intakeSubsystemMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putString("intakesubsystem State",intakesubsystemState.toString());
        SmartDashboard.putNumber("intakesubsystem Motor Speed",intakeSubsystemMotor.get());
    }
//shreydog is not a bum, shreydog is the GOAT and he's shreeging it -lik nong
    public void init() {
        intakesubsystemState = intakesubsystemStates.INTAKE_OFF;
    }

    // public void intakePowerUp(Boolean pov){
    //     if (pov) {
    //         power = power + 0.05;
    //     }
    // }
    
    // public void intakePowerDown(Boolean pov){
    //     if (pov) {
    //         power = power - 0.05;
    //     }
    // }

    public void intakeOn(double POV) {
       //power =  intakeSubsystemMotor.get();
        if (POV == 90) {
            power =+ 5;
        } else if (POV == 270){
            power =- 5;
        } else if (POV == -1){
            power = power;
        }

        intakeSubsystemMotor.set(power);
        //System.out.println("Intake On");
        intakesubsystemState = intakesubsystemState.INTAKE_ON;
    }


    public void intakeOff() {
        intakeSubsystemMotor.set(0);
        //System.out.println("Intake Off");
        intakesubsystemState = intakesubsystemState.INTAKE_OFF;
    }

    public void intakeReverse() {
        intakeSubsystemMotor.set(-0.4); //may need to be increased
        //System.out.println("Intake Reversed");
        intakesubsystemState = intakesubsystemState.INTAKE_REVERSE;
    }
}