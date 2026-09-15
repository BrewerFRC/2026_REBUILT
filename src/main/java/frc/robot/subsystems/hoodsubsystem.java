// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

//import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;
//import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
//import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.hubpositioninfo;

/**
 * Controls the shooter hood angle, including a distance-based interpolation
 * lookup so the hood auto-adjusts as the robot's range to the hub changes.
 */
public class hoodsubsystem extends SubsystemBase {
  
    public static Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    private final CommandSwerveDrivetrain drive;

    private InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();

    

    // private DigitalInput hoodlimit = new DigitalInput(Constants.OperatorConstants.hoodLimit);

    //private double drive_vely;
    //private double drive_velx;
    private double hub_xposition_blue = 4.625467;
    private double hub_yposition = 4.034663; // same for red and blue alliance
    private double hub_xposition_red = 11.915521;
    private double hub_xposition = hub_xposition_blue;

    //public TalonFX hoodstemMotor;
    //TalonFXConfiguration configs = new TalonFXConfiguration();
    public SparkMax hoodMotor;
    SparkMaxConfig config2Config = new SparkMaxConfig();
    
  
    public enum hoodSubtemStates{
        HOOD_REVERSE,    // The hoodsubsystem is reversed
        HOOD_OFF,    // The hoodsubsystem is off
        HOOD_ON;     // hoodsubsystem is on
    }
    public hoodSubtemStates hoodSubstemState = hoodSubtemStates.HOOD_OFF; //Default state
    
    /** Creates a new hoodsubsystem. */
    public hoodsubsystem(CommandSwerveDrivetrain drivetrain) {
        drive = drivetrain;

        //hoodstemMotor = new TalonFX(OperatorConstants.hoodMotorID,OperatorConstants.canbusname);
        hoodMotor = new SparkMax(OperatorConstants.hoodMotorID, MotorType.kBrushless);
        hoodMotor.getEncoder().setPosition(0);
        final double[][] shooterinterpvalues = new double[][] { {0,0}, { 2.6, 0}, { 3.29, 5.43}, {4.6, 13.4}, {5.1, 13.6} };  //(distance (meters), encoder values)
        for (double[] pair : shooterinterpvalues) {
            table.put(pair[0], pair[1]);
        }

    }
        double maxtargetangle = 13.9;

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putString("hood State",hoodSubstemState.toString());
        SmartDashboard.putNumber("hood Motor Speed",hoodMotor.get());
        SmartDashboard.putNumber("hood Angle",hoodMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean("IsHoodReady",ishoodReady());
    }

    public void init() {
        hoodSubstemState = hoodSubtemStates.HOOD_OFF;
        hoodMotor.getEncoder().setPosition(0);
    }

    public boolean hoodLowerLimit(){
        // if (!hoodlimit.get()) {
        //     hoodOff();
        //     return true;
        // } else {
        //     return false;
        // }
        return false;
    }

    private double getHubDistance() {
        /* if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            hub_xposition = hub_xposition_blue;
        } else {
            hub_xposition = hub_xposition_red;
        } */
        //Translation2d hub_position = new Translation2d(hub_xposition, hub_yposition);
        final Translation2d hub_position = hubpositioninfo.hubPosition();
        Translation2d robotPosition = drive.getState().Pose.getTranslation();
        double distance = hub_position.getDistance(robotPosition);
        //Rotation2d hubdirection = hubdirectionBlueAlliance.rotateBy(m_drivetrain.getOperatorForwardDirection());
        return distance;
    }

    public double InterpolatedHoodAngle(){  //returns angle for the hood
        return table.get(getHubDistance());
    }

    // public void hoodInterpolatedAngle(){
    //     hoodMotor.set(InterpolatedHoodPower());
    // }

    public void hoodOn() {
        double Targetangle = Math.min(InterpolatedHoodAngle(),13.6);
        if (Targetangle < maxtargetangle){
            if (Math.abs(Targetangle - hoodMotor.getEncoder().getPosition()) >= 0.1){
                if (Targetangle - hoodMotor.getEncoder().getPosition() >= 0.1){
                    hoodMotor.set(0.075);
                } else {
                    hoodMotor.set(-0.05);
                }
            } else {
                hoodMotor.set(-0.02);
            }
            //} else {
            //    hoodMotor.set(0);
            // }
        
            //hoodMotor.set(0.02);
            //System.out.println("hood On");
            hoodSubstemState = hoodSubtemStates.HOOD_ON;
        }
    }
    
    public boolean ishoodReady(){
        double Targetangle = InterpolatedHoodAngle();
        return (Math.abs(Targetangle - hoodMotor.getEncoder().getPosition()) < 0.2);
    }

    public void hoodOff() {
        hoodMotor.set(0);

        //hoodMotor.set(0);
        //System.out.println("hood Off");
        hoodSubstemState = hoodSubtemStates.HOOD_OFF;
    }

    public void hoodReverse() {
        double resetangle = 0.2;
        if (Math.abs(resetangle - Math.abs(hoodMotor.getEncoder().getPosition())) > 0.2) {
            hoodMotor.set(-0.05);
        } else {
            hoodMotor.set(0);
        }
        //System.out.println("hood Reversed");
        //hoodMotor.set(-0.05);
        hoodSubstemState = hoodSubtemStates.HOOD_REVERSE;
    }
}