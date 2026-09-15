    // Copyright (c) FIRST and other WPILib contributors.
    // Open Source Software; you can modify and/or share it under the terms of
    // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import static edu.wpi.first.units.Units.Amps;


//import java.util.Optional;


import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.Follower;
//import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
//import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;


/** Controls the feeder motor that pushes game pieces from the hopper into the shooter. */
public class feedersubsystem extends SubsystemBase {

    private final TalonFX feederMotor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private final double feederMotorRPS = 75; // was 75,60, 40, 70 RPS


    public enum feedersubsystemStates{
        FEEDER_REVERSE,    // Feeder motor is reverse
        FEEDER_ON,   // feeder is shooting
        FEEDER_OFF;     // feeder is off
    }
    public feedersubsystemStates feedersubsystemState = feedersubsystemStates.FEEDER_OFF; //Default state
    private double passivecount = 0;
        

    /** Creates a new feedersubsystem. */
    public feedersubsystem() {

        feederMotor = new TalonFX(OperatorConstants.feederMotorID,OperatorConstants.canbusname);
        final TalonFXConfiguration feederMotorconfig = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withNeutralMode(NeutralModeValue.Coast)
                                .withInverted(InvertedValue.CounterClockwise_Positive)
                )
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(50) //was 100
                                .withStatorCurrentLimitEnable(true)
                                .withSupplyCurrentLimit(40)
                                .withSupplyCurrentLimitEnable(true)
                )
                .withSlot0(
                        new Slot0Configs()
                                .withKP(0.5)
                                .withKS(0.28)
                                .withKV(0.1)
                ).withClosedLoopRamps(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.1));

        feederMotor.getConfigurator().apply(feederMotorconfig);

        // private static final TalonFXConfiguration feedersubsystemconfig = new TalonFXConfiguration()
        // .withCurrentLimits(
        //     new CurrentLimitsConfigs()
        //         // Swerve azimuth does not require much torque output, so we can set a relatively low
        //         // stator current limit to help avoid brownouts without impacting performance.
        //         .withStatorCurrentLimit(Amps.of(40))
        //         .withStatorCurrentLimitEnable(true)
        // );

        // feedersubsystemMotor.getDutyCycle().setUpdateFrequency(100);
        // feedersubsystemMotor.getTorqueCurrent().setUpdateFrequency(100);
        // feedersubsystemMotor.getMotorVoltage().setUpdateFrequency(100);

       // followerfeedersubsystemMotor.setControl(new Follower(OperatorConstants.shooterLeftMotorID, MotorAlignmentValue.Opposed).withUpdateFreqHz(100)); //might break canbus

        //feedersubsystemMotor.getConfigurator().apply(configright);
       // followerfeedersubsystemMotor.getConfigurator().apply(configleft);


        //followerfeedersubsystemMotor.setControl(new Follower(OperatorConstants.shooterLeftMotorID, MotorAlignmentValue.Opposed).withUpdateFreqHz(100)); //might break canbus

        // var talonFXConfigs = new TalonFXConfiguration();

        // var slot0Configs = talonFXConfigs.Slot0;
    
        // slot0Configs.kS = 1; // Add 0.1 V output to overcome static friction
        // slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
        // slot0Configs.kP = 3.5; // An error of 1 rotation results in 2.4 V output
        // slot0Configs.kI = 0; // no output for integrated error
        // slot0Configs.kD = 0; // A velocity of 1 rps results in 0.1 V output

        // configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        

        // feedersubsystemMotor.getConfigurator().apply(slot0Configs);

        // configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(60)).withPeakReverseTorqueCurrent(Amps.of(-60));


        // feedersubsystemMotor.getConfigurator().apply(configs);
        // followerfeedersubsystemMotor.getConfigurator().apply(configs);
        // feedersubsystemMotor.getConfigurator().apply(configs);

        //followerfeedersubsystemMotor.setControl(new Follower(OperatorConstants.preshooterMotor01ID, true));
        
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
        SmartDashboard.putString("feedersubsystem State",feedersubsystemState.toString());
        SmartDashboard.putNumber("feedersubsystem Motor Speed",feederMotor.get());
    }

    public void init() {
    

        feedersubsystemState = feedersubsystemStates.FEEDER_OFF;
    }

    public void feederOn(){

        if (isFeederOff()) {
            passivecount = 0;
        }

        if (passivecount < 25){
            passivecount = passivecount + 1;
            feederMotor.setControl(
                        velocityRequest
                                .withVelocity(-25)
                );
        } else {
        //feederMotor.set(feederMotorPower);
        feederMotor.setControl(
                velocityRequest
                        .withVelocity(feederMotorRPS)
        );
        }
        feedersubsystemState = feedersubsystemStates.FEEDER_ON;

    }

    private boolean isFeederOff(){
        if (feederMotor.get() == 0){
            return true;
        } else {
            return false;
        }
    }

    public void feederOff(){

        //feederMotor.set(0);
        feederMotor.setControl(
                velocityRequest
                        .withVelocity(0)
        );
    }


    public void feederReverse(){
        //feederMotor.set(-0.5);
        feederMotor.setControl(
                velocityRequest
                        .withVelocity(-50)
        );
        feedersubsystemState = feedersubsystemStates.FEEDER_REVERSE;
    }

    public void FeederPassiveReverse(){
        feederMotor.setControl(
                        velocityRequest
                                .withVelocity(-10)
                );
            }

}