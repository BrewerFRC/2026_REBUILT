    // Copyright (c) FIRST and other WPILib contributors.
    // Open Source Software; you can modify and/or share it under the terms of
    // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import static edu.wpi.first.units.Units.Amps;


import java.util.Optional;

import com.ctre.phoenix6.configs.*;
//import com.ctre.phoenix6.controls.Follower;
//import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
//import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.HubPositionInfo;


/**
 * Controls the shooter flywheel motors, including hub-distance-based
 * interpolation of target RPS so shot power scales with range.
 */
public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX shooterMotorRight, shooterMotorLeft;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);


    public static Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    private final CommandSwerveDrivetrain drive;

    //private TalonFXConfiguration configright = new TalonFXConfiguration();
    //private TalonFXConfiguration configleft = new TalonFXConfiguration();

    private InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();

    private double shooterMotorRPS = 50;
    //private double feederMotorPower = 0.45; // Feeder power


    //private double drive_vely;
    //private double drive_velx;
   /*  private double hub_xposition_blue = 4.625467;
    private double hub_yposition = 4.034663; // same for red and blue alliance
    private double hub_xposition_red = 11.915521; */
    private double hub_xposition = 0;

    private static final double STATOR_CURRENT_LIMIT = 60;
    private static final double SUPPLY_CURRENT_LIMIT = 50;
    private static final double KP = 0.52;
    private static final double KS = 0.24;
    private static final double KV = 0.10;
    // VelocityVoltage otherwise steps voltage almost immediately at 0.01 seconds.
    private static final double CLOSED_LOOP_RAMP_SECONDS = 0.15;


    public enum ShooterSubsystemStates{
        SHOOTER_REV,    // Revving the Preshooter, not ready to shoot
        SHOOTER_SHOOTING,   // Shooter is shooting
        SHOOTER_OFF;     // Shooter is off
    }
    public ShooterSubsystemStates ShooterSubsystemState = ShooterSubsystemStates.SHOOTER_OFF; //Default state


    /** Creates a new ShooterSubsystem. */
    public ShooterSubsystem(CommandSwerveDrivetrain drivetrain) {

        drive = drivetrain;
        shooterMotorRight = new TalonFX(OperatorConstants.SHOOTER_RIGHT_MOTOR_CAN_ID,OperatorConstants.CANIVORE_BUS);
        shooterMotorLeft = new TalonFX(OperatorConstants.SHOOTER_LEFT_MOTOR_CAN_ID,OperatorConstants.CANIVORE_BUS);

         final TalonFXConfiguration shooterRightconfig = new TalonFXConfiguration()
         .withMotorOutput(
             new MotorOutputConfigs()
                 .withNeutralMode(NeutralModeValue.Coast)
                 .withInverted(InvertedValue.Clockwise_Positive)
         )
         .withVoltage(
             new VoltageConfigs()
                     .withPeakForwardVoltage(12)
                     .withPeakReverseVoltage(0)
         )
         .withCurrentLimits(
                 new CurrentLimitsConfigs()
                     .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                     .withStatorCurrentLimitEnable(true)
                     .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                     .withSupplyCurrentLimitEnable(true)
         )
         .withSlot0(
                 new Slot0Configs()
                         .withKP(KP)
                         .withKS(KS)
                         .withKV(KV)
         ).withClosedLoopRamps(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(CLOSED_LOOP_RAMP_SECONDS));

         shooterMotorRight.getConfigurator().apply(shooterRightconfig);

        final TalonFXConfiguration shooterLeftconfig = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withNeutralMode(NeutralModeValue.Coast)
                                .withInverted(InvertedValue.CounterClockwise_Positive)
                )
                .withVoltage(
                        new VoltageConfigs()
                        .withPeakForwardVoltage(12)
                        .withPeakReverseVoltage(0)
                )
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                            .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                            .withStatorCurrentLimitEnable(true)
                            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                            .withSupplyCurrentLimitEnable(true)
                )
                .withSlot0(
                        new Slot0Configs()
                                .withKP(KP)
                         .withKS(KS)
                         .withKV(KV)
                ).withClosedLoopRamps(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(CLOSED_LOOP_RAMP_SECONDS));

        shooterMotorLeft.getConfigurator().apply(shooterLeftconfig);
        // ShooterSubsystemMotor.getDutyCycle().setUpdateFrequency(100);
        // ShooterSubsystemMotor.getTorqueCurrent().setUpdateFrequency(100);
        // ShooterSubsystemMotor.getMotorVoltage().setUpdateFrequency(100);

       // followerShooterSubsystemMotor.setControl(new Follower(OperatorConstants.SHOOTER_LEFT_MOTOR_CAN_ID, MotorAlignmentValue.Opposed).withUpdateFreqHz(100)); //might break canbus

        //ShooterSubsystemMotor.getConfigurator().apply(configright);
       // followerShooterSubsystemMotor.getConfigurator().apply(configleft);


        //followerShooterSubsystemMotor.setControl(new Follower(OperatorConstants.SHOOTER_LEFT_MOTOR_CAN_ID, MotorAlignmentValue.Opposed).withUpdateFreqHz(100)); //might break canbus

        // var talonFXConfigs = new TalonFXConfiguration();

        // var slot0Configs = talonFXConfigs.Slot0;

        // slot0Configs.kS = 1; // Add 0.1 V output to overcome static friction
        // slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
        // slot0Configs.kP = 3.5; // An error of 1 rotation results in 2.4 V output
        // slot0Configs.kI = 0; // no output for integrated error
        // slot0Configs.kD = 0; // A velocity of 1 rps results in 0.1 V output

        // configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;



        // ShooterSubsystemMotor.getConfigurator().apply(slot0Configs);

        // configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(60)).withPeakReverseTorqueCurrent(Amps.of(-60));


        // ShooterSubsystemMotor.getConfigurator().apply(configs);
        // followerShooterSubsystemMotor.getConfigurator().apply(configs);
        // FeederSubsystemMotor.getConfigurator().apply(configs);

        //followerShooterSubsystemMotor.setControl(new Follower(OperatorConstants.preshooterMotor01ID, true));
        final double[][] shooterinterpvalues = new double[][] { {0,0}, {1, 45}, { 2.6, 50}, { 3.29, 52}, {4.6, 55}, {5.1, 60} };  //(distance (meters), shooter rps)
        for (double[] pair : shooterinterpvalues) {
            table.put(pair[0], pair[1]);
        }
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
        SmartDashboard.putString("Shootersubsystem State",ShooterSubsystemState.toString());
        SmartDashboard.putNumber("Shootersubsystem Motor Speed",shooterMotorRight.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("FollowerShootersubsystem Motor Speed",shooterMotorLeft.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter Motor RPS",shooterMotorRPS);
        SmartDashboard.putNumber("Shooter Error", shooterError());
        SmartDashboard.putNumber("Shooter/Right Applied Volts", shooterMotorRight.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Right Supply Current Amps", shooterMotorRight.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Right Stator Current Amps", shooterMotorRight.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Right Temperature C", shooterMotorRight.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Right Fault Bits", shooterMotorRight.getFaultField().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Left Applied Volts", shooterMotorLeft.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Left Supply Current Amps", shooterMotorLeft.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Left Stator Current Amps", shooterMotorLeft.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Left Temperature C", shooterMotorLeft.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Left Fault Bits", shooterMotorLeft.getFaultField().getValueAsDouble());
    }

    private double shooterError(){
        return shooterMotorRight.getClosedLoopError().getValueAsDouble();
    }

    public double getHubDistance() {
        /* if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            hub_xposition = hub_xposition_blue;
        } else {
            hub_xposition = hub_xposition_red;
        } */
        final Translation2d hub_position = HubPositionInfo.hubPosition();
        Translation2d robotPosition = drive.getState().Pose.getTranslation();
        double distance = hub_position.getDistance(robotPosition);
        //Rotation2d hubdirection = hubdirectionBlueAlliance.rotateBy(m_drivetrain.getOperatorForwardDirection());
        return distance;
    }

    public double getInterpolatedShooterVelocity(){  //returns motor power
        return table.get(getHubDistance());
    }

    public boolean ShooterReady(){
        if (Math.abs(shooterError()) <= 200/60 && Math.abs(getRPS()) > 25) {  //could use shooterMotorRPS - getRPS() instead?
            return true;
        } else {
            return false;
        }
    }

    // public void ShooterPowerUp(boolean pov){
    //     if (pov) {
    //         shooterMotorRPS = shooterMotorRPS + 5;
    //     }
    // }

    // public void ShooterPowerDown(boolean pov){
    //     if (pov) {
    //         shooterMotorRPS = shooterMotorRPS - 5;
    //     }
    // }

    public void setRPS (){
        //followerShooterSubsystemMotor.setControl(new Follower(OperatorConstants.SHOOTER_RIGHT_MOTOR_CAN_ID, MotorAlignmentValue.Opposed)); //might break canbus

        //shooterMotorRight.setControl(new VelocityVoltage(shooterMotorRPS));
        //shooterMotorLeft.setControl(new V)
        shooterMotorRight.setControl(velocityRequest.withVelocity(shooterMotorRPS));
        shooterMotorLeft.setControl(velocityRequest.withVelocity(shooterMotorRPS));
    }

    public double getRPS(){
        return ((shooterMotorRight.getVelocity().getValueAsDouble()));
    }

    public void ShooterOff(){
        shooterMotorRight.set(0);
        shooterMotorLeft.set(0);
        ShooterSubsystemState = ShooterSubsystemStates.SHOOTER_OFF;
    }

    public void Shoot(int POV){
        // if (POV == 0) {
        //     shooterMotorRPS =+ 5;
        // } else if (POV == 180){
        //     shooterMotorRPS  =- 5;
        // } else if (POV == -1) {
        //     shooterMotorRPS  = shooterMotorRPS;
        // }

        setRPS();
        ShooterSubsystemState = ShooterSubsystemStates.SHOOTER_REV;

    }

    public void InterpolatedShoot(){
        shooterMotorRight.setControl(velocityRequest.withVelocity(getInterpolatedShooterVelocity()));
        shooterMotorLeft.setControl(velocityRequest.withVelocity(getInterpolatedShooterVelocity()));
        ShooterSubsystemState = ShooterSubsystemStates.SHOOTER_REV;

    }

    public void RevShooters(){
        shooterMotorRight.setControl(velocityRequest.withVelocity(shooterMotorRPS));
        shooterMotorLeft.setControl(velocityRequest.withVelocity(shooterMotorRPS));
        ShooterSubsystemState = ShooterSubsystemStates.SHOOTER_REV;
    }


}
