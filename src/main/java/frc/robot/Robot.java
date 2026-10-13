package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * WPILib's lifecycle entry point.
 *
 * <p>Keep this class small: it runs the command scheduler, transitions autonomous commands, and
 * publishes diagnostics. Hardware construction and button bindings belong in {@link RobotContainer}.
 */
public class Robot extends TimedRobot {
  private final RobotContainer robotContainer = new RobotContainer();
  private Command autonomousCommand;

  public Robot() {
    robotContainer.drivetrain.getPigeon2().setYaw(0.0);
  }

  /** Runs in every mode. The scheduler is what makes command-based bindings and defaults work. */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    publishDiagnostics();
    updateLimelightOdometry();
  }

  /** Publishes values students can compare with driver-station observations while troubleshooting. */
  private void publishDiagnostics() {
    var drivetrainState = robotContainer.drivetrain.getState();
    SmartDashboard.putBoolean(
        "Alliance", DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red);
    SmartDashboard.putNumber("PoseX", drivetrainState.Pose.getX());
    SmartDashboard.putNumber("PoseY", drivetrainState.Pose.getY());
    SmartDashboard.putNumber("Pose Rotation", drivetrainState.Pose.getRotation().getDegrees());
    SmartDashboard.putNumber("Hub Distance", robotContainer.m_shooter.getHubDistance());
    SmartDashboard.putNumber(
        "Hood Rotation", robotContainer.m_hood.hoodMotor.getEncoder().getPosition());
    SmartDashboard.putNumber(
        "Arm Encoder Value", robotContainer.m_arm.ArmSubsystemMotor.getPosition().getValueAsDouble());
    SmartDashboard.putString("Alliance Selected", DriverStation.getAlliance().toString());
    SmartDashboard.putBoolean("REBUILT Hub Active", RebuiltMatchState.isHubActive());
    SmartDashboard.putString("REBUILT Hub Phase", RebuiltMatchState.getHubPhase().toString());
    SmartDashboard.putString("REBUILT Game Data", DriverStation.getGameSpecificMessage());
  }

  /**
   * Adds a blue-origin Limelight pose only when MegaTag1 sees tags and gyro rotation is slow enough
   * that the estimate is trustworthy. Vision tuning (camera name, uncertainty, rejection rules) is
   * an on-field task and should be changed with drive-team testing.
   */
  private void updateLimelightOdometry() {
    var drivetrainState = robotContainer.drivetrain.getState();
    double headingDegrees = drivetrainState.Pose.getRotation().getDegrees();
    double rotationalVelocityRotationsPerSecond =
        Units.radiansToRotations(drivetrainState.Speeds.omegaRadiansPerSecond);

    LimelightHelpers.SetRobotOrientation("limelight-br", headingDegrees, 0, 0, 0, 0, 0);
    var measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-br");
    if (measurement != null
        && measurement.tagCount > 0
        && Math.abs(rotationalVelocityRotationsPerSecond) < 2.0) {
      robotContainer.drivetrain.addVisionMeasurement(measurement.pose, measurement.timestampSeconds);
    }
  }

  @Override
  public void disabledInit() {
    // Establish a predictable field-centric reference before drivers take control.
    robotContainer.drivetrain.getPigeon2().setYaw(0.0);
  }

  @Override
  public void disabledPeriodic() {
    SmartDashboard.putBoolean("Hood Limit", robotContainer.m_hood.hoodLowerLimit());
    SmartDashboard.putBoolean("Arm Upper Limit", robotContainer.m_arm.ArmUpperLimit());
    SmartDashboard.putBoolean("Arm Lower Limit", robotContainer.m_arm.ArmlowerLimit());
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    // Autonomous must release its subsystem requirements so driver commands can take ownership.
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(autonomousCommand);
    }
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber(
        "Pigeon Heading", robotContainer.drivetrain.getPigeon2().getYaw().refresh().getValueAsDouble());
    SmartDashboard.putNumber("Rotation Input", -robotContainer.joystick.getRightX());
    SmartDashboard.putBoolean("Shooter Ready", robotContainer.m_shooter.ShooterReady());
  }

  @Override
  public void teleopExit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testInit() {
    // Do not install test-mode default commands here: they could accidentally run a mechanism.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    SmartDashboard.putNumber("Hopper Power", robotContainer.m_HopperSubsystem.HopperSubsystemMotor.get());
  }

  @Override
  public void testExit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
