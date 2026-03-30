// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class arm extends SubsystemBase {
    TalonFX armMotor;
    private ArmStates armState = ArmStates.ARM_UP;

    /**
     * Creates a new arm.
     */
    public arm() {
        armMotor = new TalonFX(OperatorConstants.armMotorID);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putString("Arm State", armState.toString());
        SmartDashboard.putNumber("Arm Motor Speed", armMotor.get());
    }

    public void init() {
        //armMotor.setInverted(true);
        armUP();
        armState = ArmStates.ARM_UP;
    }

    public void armUP() {
        armMotor.set(0.5);
    }

    public void armDown() {
        armMotor.set(-0.5);
    }

    public void armSTOP() {
        armMotor.set(0);
    }

    public enum ArmStates {
        MOVING_UP,    // The arm is moving up
        MOVING_DOWN,    // The arm is moving down
        ARM_UP,       // arm is up
        ARM_DOWN     // arm is down

    }
}