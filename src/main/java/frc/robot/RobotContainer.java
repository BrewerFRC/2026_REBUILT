package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmDownCommand;
import frc.robot.commands.ArmStopCommand;
import frc.robot.commands.ArmUPCommand;
import frc.robot.commands.CommandCollection;
import frc.robot.commands.DriveToHubDistanceCommand;
import frc.robot.commands.FeederOffCommand;
import frc.robot.commands.FeederReverseCommand;
import frc.robot.commands.HoodOffCommand;
import frc.robot.commands.HoodOnCommand;
import frc.robot.commands.HoodReverseCommand;
import frc.robot.commands.HopperOffCommand;
import frc.robot.commands.HopperOnCommand;
import frc.robot.commands.HopperReverseCommand;
import frc.robot.commands.IntakeOffCommand;
import frc.robot.commands.IntakeOnCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.ManualHubAimDriveCommand;
import frc.robot.commands.ShootCollection;
import frc.robot.commands.ShooterOffCommand;
import frc.robot.commands.ShooterRevCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * The composition root for command-based robot code.
 *
 * <p>This is the one place that creates subsystems, maps physical controls, registers PathPlanner
 * event names, and selects autonomous routines. Subsystems contain motor behavior; commands
 * coordinate subsystems; this class connects both to the driver station.
 */
public class RobotContainer {
  /** Speed limits shared by driver-facing swerve commands. Tune after a safe practice test. */
  public static final double MAX_SPEED_METRES_PER_SECOND =
      0.7 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  public static final double MAX_ANGULAR_RATE_RADIANS_PER_SECOND =
      0.7 * RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  // These remain public because Robot publishes their diagnostic measurements to SmartDashboard.
  public final CommandXboxController joystick =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  public final CommandXboxController o_joystick =
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  public final HopperSubsystem m_HopperSubsystem = new HopperSubsystem();
  public final FeederSubsystem m_feeder = new FeederSubsystem();
  public final ShooterSubsystem m_shooter = new ShooterSubsystem(drivetrain);
  public final ArmSubsystem m_arm = new ArmSubsystem();
  public final HoodSubsystem m_hood = new HoodSubsystem(drivetrain);

  private final Telemetry telemetry = new Telemetry(MAX_SPEED_METRES_PER_SECOND);
  private final SendableChooser<Command> autoChooser;
  private final ShootCollection shootCollection =
      new ShootCollection(m_feeder, m_shooter, m_HopperSubsystem, m_hood);
  private final CommandCollection commandCollection =
      new CommandCollection(drivetrain, m_feeder, m_shooter, m_hood, m_HopperSubsystem);
  private final ManualHubAimDriveCommand manualHubAimDrive =
      new ManualHubAimDriveCommand(
          drivetrain, () -> -joystick.getLeftY(), () -> -joystick.getLeftX());
  private final DriveToHubDistanceCommand driveToHubDistance =
      new DriveToHubDistanceCommand(drivetrain);
  private final SwerveRequest.RobotCentric slowRobotCentricDrive =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public RobotContainer() {
    configureDefaultCommands();
    registerPathPlannerCommands();

    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    drivetrain.registerTelemetry(telemetry::telemeterize);
  }

  /** Sets safe resting behavior for every mechanism with a motor. */
  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(
        new TeleopDriveCommand(
            drivetrain,
            () -> -joystick.getLeftY(),
            () -> -joystick.getLeftX(),
            () -> -joystick.getRightX()));
    m_arm.setDefaultCommand(new ArmStopCommand(m_arm));
    m_IntakeSubsystem.setDefaultCommand(new IntakeOffCommand(m_IntakeSubsystem));
    m_HopperSubsystem.setDefaultCommand(new HopperOffCommand(m_HopperSubsystem));
    m_feeder.setDefaultCommand(new FeederOffCommand(m_feeder));
    m_shooter.setDefaultCommand(new ShooterOffCommand(m_shooter));
    m_hood.setDefaultCommand(new HoodOffCommand(m_hood));
  }

  /** Names commands used by PathPlanner event markers. Keep names synchronized with path files. */
  private void registerPathPlannerCommands() {
    NamedCommands.registerCommand("DeployArm", new ArmDownCommand(m_arm));
    NamedCommands.registerCommand("RetractArm", new ArmUPCommand(m_arm));
    NamedCommands.registerCommand("IntakeOn", new IntakeOnCommand(m_IntakeSubsystem));
    NamedCommands.registerCommand("IntakeOff", new IntakeOffCommand(m_IntakeSubsystem));
    NamedCommands.registerCommand("IntakeReverse", new IntakeReverseCommand(m_IntakeSubsystem));
    NamedCommands.registerCommand("HopperOn", new HopperOnCommand(m_HopperSubsystem));
    NamedCommands.registerCommand("HopperReverse", new HopperReverseCommand(m_HopperSubsystem));
    NamedCommands.registerCommand("HopperOff", new HopperOffCommand(m_HopperSubsystem));
    NamedCommands.registerCommand("ShooterOff", new ShooterOffCommand(m_shooter));
    NamedCommands.registerCommand("ShooterRev", new ShooterRevCommand(m_shooter));
    NamedCommands.registerCommand("FeederOn", new frc.robot.commands.FeederOnCommand(m_feeder));
    NamedCommands.registerCommand("HoodOn", new HoodOnCommand(m_hood));
    NamedCommands.registerCommand("HoodReverse", new HoodReverseCommand(m_hood));
    NamedCommands.registerCommand("HoodOff", new HoodOffCommand(m_hood));
    NamedCommands.registerCommand("AimAndShoot", commandCollection.aimAndShootCommand());
    NamedCommands.registerCommand("AutoShoot", shootCollection.shootCommand());
  }

  /** Documents the control map in code; see docs/controls.md for the driver-facing version. */
  private void configureBindings() {
    // Disabled drivetrain command prevents stale drive requests from moving the robot after disable.
    SwerveRequest.Idle idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // Operator: arm, collection, and a full reverse path for clearing jams.
    o_joystick.rightBumper().whileTrue(new ArmUPCommand(m_arm));
    o_joystick.leftBumper().whileTrue(new ArmDownCommand(m_arm));
    o_joystick.leftTrigger().whileTrue(new IntakeOnCommand(m_IntakeSubsystem));
    o_joystick.rightTrigger().whileTrue(new IntakeReverseCommand(m_IntakeSubsystem));
    o_joystick.rightTrigger().whileTrue(new HopperReverseCommand(m_HopperSubsystem));
    o_joystick.rightTrigger().whileTrue(new FeederReverseCommand(m_feeder));
    o_joystick.a().whileTrue(new HoodOnCommand(m_hood));
    o_joystick.x().whileTrue(new HoodReverseCommand(m_hood));

    // Driver: the POV is a slow robot-centric creep; it is useful on the charge line or in pits.
    joystick.povUp().whileTrue(
        drivetrain.applyRequest(
            () -> slowRobotCentricDrive.withVelocityX(0.5).withVelocityY(0.0)));
    joystick.povDown().whileTrue(
        drivetrain.applyRequest(
            () -> slowRobotCentricDrive.withVelocityX(-0.5).withVelocityY(0.0)));
    joystick.rightTrigger().whileTrue(shootCollection.shootCommand());
    joystick.x().whileTrue(driveToHubDistance);
    joystick.a().whileTrue(manualHubAimDrive);
    joystick.b().whileTrue(commandCollection.aimAndShootCommand());
    joystick.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
  }

  /** Returns the autonomous selected on the dashboard chooser. */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
