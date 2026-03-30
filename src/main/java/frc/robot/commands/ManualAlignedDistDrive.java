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
import frc.robot.hubpositioninfo;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;


import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public final class ManualAlignedDistDrive extends Command {
    public static Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    private final SwerveRequest.FieldCentricFacingAngle aim = new SwerveRequest.FieldCentricFacingAngle().withDeadband((TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)) * 0.1).withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.1).withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage).withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective).withHeadingPID(8, 0, 0);
    //visionsubsystem m_vision;
    private final CommandSwerveDrivetrain drive;
    double drive_velx;
    double drive_vely;
    double drive_rot;
    double hub_xposition_blue = 4.625467;
    double hub_yposition = 4.034663; // same for red and blue alliance
    double hub_xposition_red = 11.915521;
    double hub_xposition = hub_xposition_blue;
    double distance_error = 0;
    double cameraflip = 0;

    boolean distance_good;
    /**
     * Creates a new visionCommand.
     */
    public ManualAlignedDistDrive(CommandSwerveDrivetrain drivetrain, double velx, double vely, double rot) {
        drive = drivetrain;
        drive_velx = 0;
        drive_vely = 0;
        drive_rot = 0;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        /* distance_error = getDistanceerror();
        if (Math.abs(distance_error) > 2.6)
        drive.setControl(aim.withVelocityX(0.25)
                .withVelocityY(0)
                .withTargetDirection(gethubdirection())
        );
        else {
            drive.setControl(aim.withVelocityX(0)
                    .withVelocityY(0)
                    .withTargetDirection(gethubdirection())
            );
        } */
       drive.setControl(aim.withVelocityX(getDistanceerror())
                .withVelocityY(0)
                .withTargetDirection(gethubdirection()));

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
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
        return (MathUtil.isNear(MathUtil.angleModulus(targetHeading.getRadians()), MathUtil.angleModulus(currentHeading.getRadians()),Degrees.of(5).in(Radians),-Math.PI,Math.PI));
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
        
        return new Rotation2d(angle);
        //Rotation2d hubdirection = hubdirectionBlueAlliance.rotateBy(m_drivetrain.getOperatorForwardDirection());
        // return hubdirectionBlueAlliance.rotateBy(drive.getOperatorForwardDirection() + 180);
    }

    private double getDistanceerror() {
        Translation2d robotPosition = drive.getState().Pose.getTranslation();
        Translation2d hub_position = new Translation2d(hub_xposition, hub_yposition);
        double distance_error = Math.sqrt((Math.pow((hub_position.getY() - robotPosition.getY()),2)+Math.pow((hub_position.getX() - robotPosition.getX()),2)))-2.6;
        SmartDashboard.putNumber("Aligned Drive Distance Error", distance_error);
        SmartDashboard.putBoolean("Distance Ready", distance_good);
        if (distance_error < 0.1){
            drive_velx = 0;
            distance_good = true;
        } else { 
            drive_velx = Math.max(0.6,2.5*distance_error);
            distance_good = false;
        }
        return drive_velx;
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