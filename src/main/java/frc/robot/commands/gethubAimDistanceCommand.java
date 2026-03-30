// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;
import frc.robot.hubpositioninfo;


import java.util.Optional;

import static edu.wpi.first.units.Units.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class gethubAimDistanceCommand extends Command {
    //double hub_xposition;
    public static Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    private final SwerveRequest.FieldCentricFacingAngle aim = new SwerveRequest.FieldCentricFacingAngle().withDeadband((TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)) * 0.1).withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.1).withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage).withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective).withHeadingPID(8, 0, 0);
    //visionsubsystem m_vision;
    private final CommandSwerveDrivetrain drive;
    double drive_velx;
    double drive_vely;
    double hub_xposition_blue = 4.625467;
    double hub_yposition = 4.034663; // same for red and blue alliance
    double hub_xposition_red = 11.915521;
    public double hub_xposition = hub_xposition_blue;
    double cameraflip = 0;
    double lockeddirection = 0;
    boolean alignedflag = false;

    /**
     * Creates a new visionCommand.
     */
    public gethubAimDistanceCommand(CommandSwerveDrivetrain drivetrain, double velx, double vely) {
        //m_vision = param_vision;
        drive = drivetrain;
        drive_velx = velx;
        drive_vely = vely;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        lockeddirection = 0;
        alignedflag = false;
        // if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
        //     hub_xposition = hub_xposition_blue;
        //     //cameraflip = Math.PI;
        //     cameraflip = 0;
        // } else { 
        //     hub_xposition = hub_xposition_red;
        //     cameraflip = 0;
        // }
        /* if (shouldFlip()) {
            hub_xposition = hub_xposition_red;
        } else {
            hub_xposition = hub_xposition_blue;
        } */
    }

    /* public static boolean shouldFlip() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    } */

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
       /*  if (true) {
            var driveState = drive.getState();
            double headingDeg = driveState.Pose.getRotation().getDegrees();
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

            LimelightHelpers.SetRobotOrientation("limelight-br", headingDeg, 0, 0, 0, 0, 0);
            var llMeasurement2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-br");
            var llMeasurement1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-br");
            var llmeasurement = Pose2d(llMeasurement2.pose.getTranslation(),llMeasurement1.pose.getRotation());
            if (llMeasurement2 != null && llMeasurement2.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
                drive.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
            }
        } */
        drive.setControl(aim.withVelocityX(drive_velx)
                .withVelocityY(drive_vely)
                .withTargetDirection(gethubdirection())
        );

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        alignedflag = false;
        lockeddirection = 0;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public boolean robotaligned() {

            Rotation2d targetHeading = aim.TargetDirection;
            Rotation2d currentHeadingBlueAlliance = drive.getState().Pose.getRotation();
            Rotation2d currentHeading = currentHeadingBlueAlliance.rotateBy(drive.getOperatorForwardDirection());
            if (MathUtil.isNear(MathUtil.angleModulus(targetHeading.getRadians()), MathUtil.angleModulus(currentHeading.getRadians()),Degrees.of(3).in(Radians),-Math.PI,Math.PI)) {
                lockeddirection = targetHeading.getRadians();
                alignedflag=true;
            }
            return (MathUtil.isNear(MathUtil.angleModulus(targetHeading.getRadians()), MathUtil.angleModulus(currentHeading.getRadians()),Degrees.of(3).in(Radians),-Math.PI,Math.PI));
    }

    private Rotation2d gethubdirection() {
        /* if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            hub_xposition = hub_xposition_blue;
        } else { 
            hub_xposition = hub_xposition_red;
        }
 */

        hub_xposition = hubpositioninfo.hubPosition().getX(); 
        SmartDashboard.putNumber("hub_xposition", hub_xposition);
        SmartDashboard.putNumber("angleflip",hubpositioninfo.cameraflip());
        

        final Translation2d hub_position = hubpositioninfo.hubPosition();

        Translation2d robotPosition = drive.getState().Pose.getTranslation();
        // robotPosition.addVisionMeasurement(17,4);
         Rotation2d hubdirectionBlueAlliance = hub_position.minus(robotPosition).getAngle();
        SmartDashboard.putNumber("HubDirectionBlueAllianceAngle",hubdirectionBlueAlliance.getRadians());
        double angle = Math.atan2(hub_position.getY() - robotPosition.getY(),hub_position.getX() - robotPosition.getX());
        SmartDashboard.putNumber("angle",angle);
        // if (hub_xposition == hub_xposition_blue){
        //     cameraflip = Math.PI;
        // } else {
        //     cameraflip = 0;
        // }
        angle = hubpositioninfo.cameraflip()+Math.atan2(hub_position.getY() - robotPosition.getY(),hub_position.getX() - robotPosition.getX());
        SmartDashboard.putNumber("revised angle", angle);
        SmartDashboard.putNumber("LockedDirection",lockeddirection);
        SmartDashboard.putBoolean("Aligned Flag", alignedflag);
        if (true){
            if (alignedflag==true && Math.abs(angle-lockeddirection)>5*Math.PI/180) {
                angle = lockeddirection;
            }
        }
    return new Rotation2d(angle);
        //Rotation2d hubdirection = hubdirectionBlueAlliance.rotateBy(m_drivetrain.getOperatorForwardDirection());
        // return hubdirectionBlueAlliance.rotateBy(drive.getOperatorForwardDirection() + 180);
    }
}
/*
        aiming ctre swerve
        To aim at moving targets or specific field elements like the Speaker or Reef:

        Update Pose: Use addVisionMeasurement() in your SwerveDrivetrain class to fuse vision data with odometry.
        This ensures the robot knows exactly where it is and where the target is.
        Calculate Heading: Use Math.atan2(targetY - robotY, targetX - robotX)
        to find the target angle. Pass this angle into a FieldCentricFacingAngle
        request to keep the robot pointed at the target while you drive.  */