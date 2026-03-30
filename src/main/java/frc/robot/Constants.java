// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
  public final class Constants {

    public static class FieldConstants {
      
    }

    public static class OperatorConstants {

      //PWM ID's
      public static int hopperMotorID = 3;     // (HotDog Rollers) Spark
      //public static int hopperMotorID = 58; //CAN SparkMax
      
      // public static int upperlimitID = 0;
      // public static int lowerlimitID01 = 1;
      // public static int lowerlimitID02 = 2;
      // public static int hoodLimit = 9;

      public static CANBus canbusname = new CANBus("4564_Canivore_01");

      //smartdashboard ports
      public static int kDriverControllerPort = 0;
      public static int kOperatorControllerPort = 1;
        
      public static int armMotorID = 51;         //Falcon 500 
      public static int intakeMotorID = 52;      //NEO Vortex
      //public static int feederMotorID = 53;      //Falcon 
      //public static int preshooterMotor01ID = 54;//Kraken
      //public static int preshooterMotor02ID = 55;//Kraken
      public static int shooterLeftMotorID = 56; //Kraken X60
      public static int shooterRightMotorID = 57; //Kraken X60
      public static int feederMotorID = 53; //Kraken X44
      public static int hoodMotorID = 59;        //ID??? Falcon/Kraken?

      public static double loopPeriodSecs = 0.02;// Robot's control loop time

      public static int kAngleTolerance = 1;


    }

    
  }
  
