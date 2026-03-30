    // Copyright (c) FIRST and other WPILib contributors.
    // Open Source Software; you can modify and/or share it under the terms of
    // the WPILib BSD license file in the root directory of this project.

    package frc.robot;

    import java.util.Optional;
    
    import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

//import com.ctre.phoenix6.HootAutoReplay;

    import edu.wpi.first.math.util.Units;
    import edu.wpi.first.wpilibj.DriverStation;
    import edu.wpi.first.wpilibj.DriverStation.Alliance;
    import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.Command;
    import edu.wpi.first.wpilibj2.command.CommandScheduler;
    //import frc.robot.commands.ArmDownCommand;
    //import frc.robot.commands.ArmUPCommand;
    //import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
    // import frc.robot.subsystems.shootersubsystem;
    // import frc.robot.subsystems.intakesubsystem;
    // import frc.robot.Constants.OperatorConstants;
    // import frc.robot.commands.ArmDownCommand;
    // import frc.robot.commands.ArmStopCommand;
    // import frc.robot.commands.ArmUPCommand;
    import frc.robot.commands.HopperOnCommand;
    //import frc.robot.commands.IntakeOnCommand;
    //import frc.robot.commands.ManualAlignedDrive;
    //import frc.robot.commands.ShootCommand;
    //import frc.robot.commands.ShooterRevCommand;
    //import frc.robot.generated.TunerConstants;
    //import frc.robot.subsystems.CommandSwerveDrivetrain;
    //import frc.robot.subsystems.armsubsystem;
    //import frc.robot.subsystems.feedersubsystem;
    // import frc.robot.commands.IntakeOnCommand;
    // import frc.robot.commands.IntakeOffCommand;
    // import frc.robot.subsystems.armsubsystem;
    // import frc.robot.subsystems.hoppersubsystem;
    //import frc.robot.subsystems.hoodsubsystem;
    //import frc.robot.subsystems.hoppersubsystem;
    //import frc.robot.subsystems.intakesubsystem;
    //import frc.robot.subsystems.shootersubsystem;
    
    public class Robot extends TimedRobot {
        private Command m_autonomousCommand;
        Translation2d translation = new Translation2d(0,0);
        Rotation2d rotation = new Rotation2d(0);
        // public double hub_xposition_blue = 4.625467;
        // public double hub_yposition = 4.034663;
        // public double hub_xposition_red = 11.915521;
        // public double hub_xposition = 0;
        // public static Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        
        private final RobotContainer m_robotContainer;

        
        // public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        // public final ManualAlignedDrive manualAlignedDrive = new ManualAlignedDrive(drivetrain, null, null, null);

        // public final intakesubsystem m_intakesubsystem = new intakesubsystem();

        // public final hoppersubsystem m_hoppersubsystem = new hoppersubsystem();

        // 	public final feeder m_feeder = new feeder();

        // public final shootersubsystem m_shooter = new shootersubsystem(drivetrain);
        
        // public final armsubsystem m_arm = new armsubsystem();

        // public final hoodsubsystem m_hood = new hoodsubsystem(drivetrain);

        
        //public final CommandXboxController o_joystick = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

        //private int time;

        /* log and replay timestamp and joystick data */
        // private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        //     .withTimestampReplay()
        //     .withJoystickReplay();




        public Robot() {
            m_robotContainer = new RobotContainer();
            m_robotContainer.drivetrain.getPigeon2().setYaw(0);

        }

        private final SendableChooser<Command> m_chooser = new SendableChooser<>();

        @Override
        public void robotPeriodic() {
            //m_timeAndJoystickReplay.update();
            CommandScheduler.getInstance().run();
            SmartDashboard.putBoolean("Alliance", DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red);

            SmartDashboard.putNumber("PoseX", m_robotContainer.drivetrain.getState().Pose.getX());
            SmartDashboard.putNumber("PoseY", m_robotContainer.drivetrain.getState().Pose.getY());
            SmartDashboard.putNumber("Pose Rotation", m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
            SmartDashboard.putNumber("Hub Distance", m_robotContainer.m_shooter.getHubDistance());
            SmartDashboard.putNumber("Hood Rotation",m_robotContainer.m_hood.hoodMotor.getEncoder().getPosition());

            SmartDashboard.putNumber("Arm Encoder Value", m_robotContainer.m_arm.armsubsystemMotor.getPosition().getValueAsDouble());
            SmartDashboard.putString("Alliance Selected", DriverStation.getAlliance().toString());

            //m_chooser.setDefaultOption("Blue", "Blue");
            //m_chooser.addOption("Blue", Red());

            // Publish to SmartDashboard
            //SmartDashboard.putData("Aliance Color", "Color");

            // SmartDashboard.putNumber("hud dist", hub_xposition);

            /*
            * This example of adding Limelight is very simple and may not be sufficient for on-field use.
            * Users typically need to provide a standard deviation that scales with the distance to target
            * and changes with number of tags available.
            *
            * This example is sufficient to show that vision integration is possible, though exact implementation
            * of how to use vision should be tuned per-robot and to the team's specification.
            */



            // if (DriverStation.getAlliance().isPresent()) {
            //     if (DriverStation.getAlliance().get() == Alliance.Red) {
            //         hub_xposition = hub_xposition_red;
            //     }
            //     if (DriverStation.getAlliance().get() == Alliance.Blue) {
            //         hub_xposition = hub_xposition_blue;
            //     }
            // }
            // else {     
            // }
            // if (true) {
            //     var driveState = m_robotContainer.drivetrain.getState();
            //     double headingDeg = driveState.Pose.getRotation().getDegrees();
            //     double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

            //     LimelightHelpers.SetRobotOrientation("limelight-br", headingDeg, 0, 0, 0, 0, 0);
            //     var llMeasurement1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-br"); // MegaTag1 for rotation
            //     var llMeasurement2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-br");// MegaTag2 for translation

                // try {
                //     if (llMeasurement1 != null && llMeasurement1.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
                //         rotation = llMeasurement1.pose.getRotation();
                //     } else {
                //         rotation = llMeasurement2.pose.getRotation();
                //     }
                    
                //     if (llMeasurement2 != null && llMeasurement2.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
                //         translation = llMeasurement2.pose.getTranslation();
                //     } else {
                //         translation = llMeasurement1.pose.getTranslation();
                //     }

                //     if (llMeasurement2.tagCount > 0 || llMeasurement1.tagCount>0) {     
                //         llMeasurement2.pose = new Pose2d(translation, rotation);
                //         m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement2.pose, llMeasurement2.timestampSeconds);
                //     }
                // } catch (Exception e) {};

                // if (llMeasurement1 != null && llMeasurement1.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
                //     rotation = llMeasurement1.pose.getRotation();
                // } else {
                //     rotation = llMeasurement2.pose.getRotation();
                // }
                
                // if (llMeasurement2 != null && llMeasurement2.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
                //     translation = llMeasurement2.pose.getTranslation();
                // }
                // llMeasurement2.pose = new Pose2d(translation, rotation);
                // m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement2.pose, llMeasurement2.timestampSeconds);
            // }


            if (true) {
                var driveState = m_robotContainer.drivetrain.getState();
                double headingDeg = driveState.Pose.getRotation().getDegrees();
                double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

                LimelightHelpers.SetRobotOrientation("limelight-br", headingDeg, 0, 0, 0, 0, 0);
                var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-br");
                var llMeasurement2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-br");
        
                if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
                    m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
                } 
            }
           /*  if (true) {
                var driveState = m_robotContainer.drivetrain.getState();
                double headingDeg = driveState.Pose.getRotation().getDegrees();
                double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

                LimelightHelpers.SetRobotOrientation("limelight-br", headingDeg, 0, 0, 0, 0, 0);
                var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-br");
                if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
                    m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
                }
            } */
        }

        @Override
        public void disabledInit() {
            m_robotContainer.drivetrain.getPigeon2().setYaw(0);
        }

        @Override
        public void disabledPeriodic() {
            SmartDashboard.putBoolean("Hood Limit", m_robotContainer.m_hood.hoodLowerLimit());
            SmartDashboard.putBoolean("Arm Upper Limit", m_robotContainer.m_arm.ArmUpperLimit());
            SmartDashboard.putBoolean("Arm Lower Limit", m_robotContainer.m_arm.ArmlowerLimit());
            
            SmartDashboard.putNumber("PoseX", m_robotContainer.drivetrain.getState().Pose.getX());
            SmartDashboard.putNumber("PoseY", m_robotContainer.drivetrain.getState().Pose.getY());
            SmartDashboard.putNumber("Pose Rotation", m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
            SmartDashboard.putNumber("Hub Distance", m_robotContainer.m_shooter.getHubDistance());
            SmartDashboard.putNumber("Hood Rotation",m_robotContainer.m_hood.hoodMotor.getEncoder().getPosition());
            //SmartDashboard.putNumber("HUB X POSITION",m_robotContainer.gethubAimDistanceCommand().hub_xposition);
        }

        @Override
        public void disabledExit() {}

        @Override
        public void autonomousInit() {
            m_autonomousCommand = m_robotContainer.getAutonomousCommand();

            if (m_autonomousCommand != null) {
                CommandScheduler.getInstance().schedule(m_autonomousCommand);
            }
        }

        @Override
        public void autonomousPeriodic() {}

        @Override
        public void autonomousExit() {}

        @Override
        public void teleopInit() {
            if (m_autonomousCommand != null) {
                CommandScheduler.getInstance().cancel(m_autonomousCommand);
            }

        
            
        }

        @Override
        public void teleopPeriodic() {
            SmartDashboard.putNumber("Pigeon Heading", m_robotContainer.drivetrain.getPigeon2().getYaw().refresh().getValueAsDouble());
            SmartDashboard.putNumber("Rotation Input",-m_robotContainer.joystick.getRightX());

            SmartDashboard.putData(m_autonomousCommand);
            SmartDashboard.putBoolean("Shooter Ready", m_robotContainer.m_shooter.ShooterReady());
        }

        @Override
        public void teleopExit() {
            CommandScheduler.getInstance().cancelAll();
        }

        @Override
        public void testInit() {
            CommandScheduler.getInstance().cancelAll();

        // time = 0;

            /*m_robotContainer.hopper.init();
            RobotContainer.m_intake.init();
            RobotContainer.m_shooter.init();*/
        }

        @Override
        public void testPeriodic() {
        

            SmartDashboard.putBoolean("Hood Limit", m_robotContainer.m_hood.hoodLowerLimit());
            SmartDashboard.putBoolean("Arm Upper Limit", m_robotContainer.m_arm.ArmUpperLimit());
            SmartDashboard.putBoolean("Arm Lower Limit", m_robotContainer.m_arm.ArmlowerLimit());
            SmartDashboard.putNumber("Hopper Power", m_robotContainer.m_hoppersubsystem.hoppersubsystemMotor.get());

            // o_joystick.leftTrigger().whileTrue(new IntakeOnCommand(m_intakesubsystem));
            // o_joystick.povDown().whileFalse(new IntakeOffCommand(m_intakesubsystem));

            // o_joystick.rightBumper().whileTrue(new ArmUPCommand(m_armsubsytem));
            // o_joystick.leftBumper().whileTrue(new ArmDownCommand(m_armsubsytem));

            //  m_robotContainer.m_intakesubsystem.setDefaultCommand(
            //      new IntakeOnCommand(m_robotContainer.m_intakesubsystem)
            //  );

            m_robotContainer.m_hoppersubsystem.setDefaultCommand(
                new HopperOnCommand(m_robotContainer.m_hoppersubsystem)
            );

            // m_robotContainer.m_shooter.setDefaultCommand(
            //     new ShooterRevCommand(m_robotContainer.m_shooter, -1)
            // );

            // m_robotContainer.m_shooter.setDefaultCommand(
            //     new ShootCommand(m_robotContainer.m_shooter)
            // );

            // m_robotContainer.m_arm.setDefaultCommand(
            //     new ArmUPCommand(m_robotContainer.m_arm)
            // );

            // // m_robotContainer.m_arm.setDefaultCommand(
            // //     new ArmDownCommand(m_robotContainer.m_arm)
            // // );

            // m_robotContainer.m_intakesubsystem.setDefaultCommand(
            //     new IntakeOnCommand(m_robotContainer.m_intakesubsystem)
            // );

            //o_joystick.a().whileTrue(new IntakeReverseCommand(m_intakesubsystem));

            //o_joystick.b().whileTrue(new HopperReverseCommand(m_hoppersubsystem)); 
            

            //joystick.x().whileTrue(m_robotContainer.commandcollection.AimAndShootCommand());


            //m_robotContainer.m_hoppersubsystem.hoppersubsystemMotor.set(0.9);
            //m_robotContainer.m_hoppersubsystem.setDefaultCommand(new HopperOnCommand(m_robotContainer.m_hoppersubsystem));
            //m_robotContainer.m_intakesubsystem.setDefaultCommand(new IntakeOnCommand(m_robotContainer.m_intakesubsystem));
            ///m_shooter.shootersubsystemMotor.set(0);
            // time = time + 1;
            // if (time >= 10) {
            //     m_shooter.feedersubsystemMotor.set(0.3);
            // }


            //SmartDashboard.putData("HotDogRoller Power", m_hopper.hoppersubsystemMotor);
            /*SmartDashboard.putData("PreShooter Power", m_shooter.shootersubsystemMotor);
            SmartDashboard.putData("Feeder Power", m_shooter.feedersubsystemMotor);*/


        }

        @Override
        public void testExit() {
            CommandScheduler.getInstance().cancelAll();
        }

        @Override
        public void simulationPeriodic() {}
    }
