// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.Constants.OperatorConstants;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
//import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.*;


public class RobotContainer {
  //  public final ArmUPCommand m_armUP = new ArmUPCommand(m_arm);

    double shooterMotorRPS;
    
    SlewRateLimiter filterX = new SlewRateLimiter(0.8);
    SlewRateLimiter filterY = new SlewRateLimiter(0.8);
    SlewRateLimiter filterTurn = new SlewRateLimiter(0.8);    
    
    public static double MaxSpeed = 0.7 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = 0.7 * RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private Command m_autonomousCommand;
    public double hub_xposition_blue = 4.625467;
    public double hub_yposition = 4.034663;
    public double hub_xposition_red = 11.915521;
    public double hub_xposition = 0;

    
    //private double currdrivexinput = 0;
    //private double currdriveyinput = 0;
    //private double currrotinput = 0;

    /* Setting up bindings for necessary control of the swerve drive platform */
    /* private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband to maxSpeed
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);  */// Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //private final CommandXboxController joystick = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    public final CommandXboxController joystick = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    public final CommandXboxController o_joystick = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //public final ManualAlignedDrive manualAlignedDrive = new ManualAlignedDrive(drivetrain, null, null, null);

    public final intakesubsystem m_intakesubsystem = new intakesubsystem();

    public final hoppersubsystem m_hoppersubsystem = new hoppersubsystem();

    public final feedersubsystem m_feeder = new feedersubsystem();

    public final shootersubsystem m_shooter = new shootersubsystem(drivetrain);
    
    public final armsubsystem m_arm = new armsubsystem();

    public final hoodsubsystem m_hood = new hoodsubsystem(drivetrain);


    //public final feeder feed = new feeder();



    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public final ShootCollection shootCollection = new ShootCollection(
        m_feeder,
        m_shooter,
        m_hoppersubsystem,
        m_hood
    );

    public final commandcollection commandcollection = new commandcollection(
        drivetrain,
        m_intakesubsystem,
        m_feeder,
        m_shooter,
        m_hood,
        m_hoppersubsystem,
        () -> -joystick.getLeftY() * MaxSpeed,
        () -> -joystick.getLeftX() * MaxSpeed
    );

    final ManualAlignedMoveCommand ManualAlignedMove = new ManualAlignedMoveCommand(
        drivetrain, 
        () -> -joystick.getLeftY(),
        () -> -joystick.getLeftX(),
        () -> -joystick.getRightX()
        );
    

    public final ManualAlignedDistDrive ManualAlignedDistDrive = new ManualAlignedDistDrive(drivetrain, 0, 0, 0);

    public RobotContainer() {
        
            /* if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get() == Alliance.Red) {
                    hub_xposition = hub_xposition_red;
                }
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    hub_xposition = hub_xposition_blue;
                }
            }
            else {     
            }
 */


        //Named path planner commands
        m_arm.setDefaultCommand(new ArmStopCommand(m_arm));
        NamedCommands.registerCommand("DeployArm", new ArmDownCommand(m_arm));
        NamedCommands.registerCommand("RetractArm", new ArmUPCommand(m_arm));

        new EventTrigger("DeployArm").whileTrue(new ArmDownCommand(m_arm));

        m_intakesubsystem.setDefaultCommand(new IntakeOffCommand(m_intakesubsystem));
        NamedCommands.registerCommand("IntakeOn", new IntakeOnCommand(m_intakesubsystem, -1));
        NamedCommands.registerCommand("IntakeOff", new IntakeOffCommand(m_intakesubsystem));
        NamedCommands.registerCommand("IntakeReverse", new HopperReverseCommand(m_hoppersubsystem));

        new EventTrigger("IntakeOn").whileTrue(new IntakeOnCommand(m_intakesubsystem, -1));

        m_hoppersubsystem.setDefaultCommand(new HopperOffCommand(m_hoppersubsystem));
        NamedCommands.registerCommand("HopperOn", new HopperOnCommand(m_hoppersubsystem));
        NamedCommands.registerCommand("HopperReverse", new HopperReverseCommand(m_hoppersubsystem));
        NamedCommands.registerCommand("HopperOff", new HopperOffCommand(m_hoppersubsystem));      
        
        m_shooter.setDefaultCommand(new ShooterOffCommand(m_shooter));
        NamedCommands.registerCommand("ShooterOff", new ShooterOffCommand(m_shooter));
        NamedCommands.registerCommand("ShooterRev", new ShooterRevCommand(m_shooter));
        NamedCommands.registerCommand("ShooterOn", new ShootCommand(m_shooter));
        
        m_feeder.setDefaultCommand(new FeederOffCommand(m_feeder));
        NamedCommands.registerCommand("FeederOn", new FeederOnCommand(m_feeder));        
        
        m_hoppersubsystem.setDefaultCommand(new HopperOffCommand(m_hoppersubsystem));
        NamedCommands.registerCommand("HopperOn", new HopperOnCommand(m_hoppersubsystem));

        m_hood.setDefaultCommand(new HoodReverseCommand(m_hood));
        NamedCommands.registerCommand("HoodReverse", new HoodReverseCommand(m_hood));
        NamedCommands.registerCommand("HoodOff", new HoodOffCommand(m_hood));

        NamedCommands.registerCommand("AimAndShoot", commandcollection.AimAndShootCommand());

        NamedCommands.registerCommand("AutoShoot", shootCollection.ShootCommand());

       // NamedCommands.registerCommand("DeployArm", Commands.runOnce(()-> new ArmUPCommand(m_arm)));
        //NamedCommands.registerCommand("Text", Commands.runOnce(()-> {System.out.println("I'm deployed!");}));

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        //FollowPathCommand.warmupCommand().schedule();
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
        SmartDashboard.putData("Command Instance", CommandScheduler.getInstance());

        SmartDashboard.putString("DriverStation Alliance", DriverStation.getAlliance().toString());
        SmartDashboard.putNumber("hud dist", hub_xposition);

    }

    

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        configureteleopdriveBindings();
 /*
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(((filterY.calculate(-joystick.getLeftY())) * MaxSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY(((filterX.calculate(-joystick.getLeftX())) * MaxSpeed)) // Drive left with negative X (left)
                    .withRotationalRate((filterTurn.calculate(-joystick.getRightX())) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
*/

/*      double getLeftY = -joystick.getLeftY();
        double getLeftX = -joystick.getLeftX();
        double getRightX = -joystick.getRightX();
        double putLeftY = ramp(currdriveyinput,getLeftY,0.1);
        double putLeftX = ramp(currdrivexinput,getLeftX,0.1);
        double putRightX = ramp(currrotinput,getRightX,0.05);
        currdriveyinput = putLeftY;
        currdrivexinput = putLeftX;
        currrotinput = putRightX;
       drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() ->
                        drive.withVelocityX(putLeftY * MaxSpeed) // Drive forward with negative Y (forward)
                                .withVelocityY(putLeftX * MaxSpeed) // Drive left with negative X (left)
                                .withRotationalRate(putRightX * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
        );
*/
        m_arm.setDefaultCommand(
            new ArmStopCommand(m_arm)
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        RobotModeTriggers.disabled().whileTrue(
            new ArmStopCommand(m_arm)
        );

        //shooterMotorRPS = 50;
        // joystick.leftTrigger().whileTrue(new ShooterRevCommand(m_shooter));  //Rev and shoot stuff
        // joystick.leftTrigger().whileFalse(new ShooterOffCommand(m_shooter));
        // joystick.rightTrigger().onTrue(new ShootCommand(m_shooter));

        // if (m_shooter.ShooterReady() && joystick.rightTrigger().getAsBoolean()){

        //     new HopperOnCommand(m_hoppersubsystem);
        //     new FeederOnCommand(m_shooter);
        // }

            // joystick.rightTrigger().whileTrue(new FeederOnCommand(m_feeder));
            // joystick.rightTrigger().whileTrue(new HopperOnCommand(m_hoppersubsystem));           

        o_joystick.a().onTrue(new HoodOnCommand(m_hood));
        o_joystick.a().whileFalse(new HoodOffCommand(m_hood));
        o_joystick.x().whileTrue(new HoodReverseCommand(m_hood));
        o_joystick.x().whileFalse(new HoodOffCommand(m_hood));

        // joystick.rightTrigger().whileFalse(new HopperOffCommand(m_hoppersubsystem));
        // joystick.rightTrigger().whileFalse(new FeederOffCommand(m_feeder));

        // joystick.button(1).whileTrue(new ArmStopCommand(m_arm));
        //joystick.x().whileTrue(new visionCommand(m_visionsubsystem));
        //joystick.y().onTrue(drivetrain.applyRequest(()-> getPigeon2().setYaw(0)));

        // new IntakePowerUpCommand(m_intakesubsystem, o_joystick.povRight().getAsBoolean());
        // new IntakePowerDownCommand(m_intakesubsystem, o_joystick.povLeft().getAsBoolean());

        // new ShooterPowerUpCommand(m_shooter, o_joystick.povUp().getAsBoolean());
        // new ShooterPowerDownCommand(m_shooter, o_joystick.povDown().getAsBoolean());

        // o_joystick.pov(90).whileTrue(new IntakePowerUpCommand(m_intakesubsystem));
        // o_joystick.pov(270).whileTrue(new IntakePowerDownCommand(m_intakesubsystem));

        //o_joystick.a().whileTrue(new IntakeReverseCommand(m_intakesubsystem));

        //o_joystick.b().whileTrue(new HopperReverseCommand(m_hoppersubsystem)); 
        

        //joystick.x().whileTrue(commandcollection.AimAndShootCommand());
        // joystick.x().whileFalse(new HoodReverseCommand(m_hood));
        //joystick.x().whileFalse(new HopperOffCommand(m_hoppersubsystem));
        //joystick.rightTrigger().onTrue(Commands.runOnce(() -> ManualAlignedDrive()));

        // o_joystick.b().toggleOnTrue(new HopperOnCommand(m_hoppersubsystem));
  
        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Operator
        o_joystick.rightBumper().whileTrue(new ArmUPCommand(m_arm));
        o_joystick.leftBumper().whileTrue(new ArmDownCommand(m_arm));

        o_joystick.leftTrigger().whileTrue(new IntakeOnCommand(m_intakesubsystem, o_joystick.getHID().getPOV()));

        o_joystick.rightTrigger().whileTrue(new IntakeReverseCommand(m_intakesubsystem, o_joystick.getHID().getPOV()));
        o_joystick.rightTrigger().whileTrue(new HopperReverseCommand(m_hoppersubsystem));
        o_joystick.rightTrigger().whileTrue(new FeederReverseCommand(m_feeder));

        //Driver
        joystick.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        joystick.rightTrigger().whileTrue(shootCollection.ShootCommand());

        joystick.x().whileTrue(ManualAlignedDistDrive);
        joystick.a().whileTrue(ManualAlignedMove);
        joystick.b().whileTrue(commandcollection.AimAndShootCommand());
        joystick.b().whileFalse(new HoodReverseCommand(m_hood));
        //joystick.x().debounce(0.2).toggleOnTrue(ManualAlignedMove);
        //joystick.x().debounce(0.2).toggleOnFalse(ManualAlignedDistDrive);
        //joystick.x().whileTrue(commandcollection.AimAndShootCommand());

        //joystick.b().whileTrue(new FeederReverseCommand(m_feeder));
        //joystick.b().whileFalse(new FeederOffCommand(m_feeder));

        //joystick.b().whileTrue(new HopperReverseCommand(m_hoppersubsystem));
        // joystick.b().whileFalse(new HopperReverseCommand(m_hoppersubsystem));

        // Reset the field-centric heading on left bumper press.
        joystick.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    private void configureteleopdriveBindings() {
        final teleopdriveCommand teleopdrive = new teleopdriveCommand(
        drivetrain, 
        () -> -joystick.getLeftY(),
        () -> -joystick.getLeftX(),
        () -> -joystick.getRightX()
        );
    
        drivetrain.setDefaultCommand(teleopdrive);
    }


   /*  public static double ramp(double currentValue, double targetValue, double rampRate){
        if(currentValue < targetValue){
            currentValue = currentValue + rampRate;
            if(currentValue > targetValue){
                currentValue = targetValue;
            }
        }
        if(currentValue > targetValue){
            currentValue = currentValue - rampRate;
            if(currentValue < targetValue){
                currentValue = targetValue;
            }
        }
        return currentValue;
    } */

}
