package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import frc.robot.hubpositioninfo;

/**
 * Manual driving with the heading locked onto the hub: translation still
 * comes from the joystick, but rotation is overridden to face the hub using
 * {@link #gethubdirection()}.
 */
public class ManualAlignedMoveCommand extends Command {

  public double hub_xposition = 0;
    private final CommandSwerveDrivetrain drive;
    private DoubleSupplier getLeftY;
    private DoubleSupplier getLeftX;
    private DoubleSupplier getRightX;
    private double drive_ramprate = 0.02;
    private double turn_ramprate = 0.04;
    private double currdrivexinput = 0;
    private double currdriveyinput = 0;
    private double currrotinput = 0;
    private double inputtracker = 0;

    public static double MaxSpeed = 0.7*TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentricFacingAngle aim2 = new SwerveRequest.FieldCentricFacingAngle()
            .withRotationalDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective)
            .withHeadingPID(8, 0, 0);

    public ManualAlignedMoveCommand(CommandSwerveDrivetrain commandSwerveDrivetrain, DoubleSupplier getLeftY, DoubleSupplier getLeftX, DoubleSupplier getRightX) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.drive = commandSwerveDrivetrain;
        this.getLeftY = getLeftY;
        this.getLeftX = getLeftX;
        this.getRightX = getRightX;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        currdrivexinput = 0;
        currdriveyinput = 0;
        currrotinput = 0;
    }

    @Override
    public void execute() {

        if (true) {//(isrobotaligned()) {
            if (getLeftX.getAsDouble() == 0 && getLeftY.getAsDouble() == 0) {
                inputtracker = inputtracker + 1;
            }
            else {
                inputtracker = 0;
            }
            if (inputtracker <= 2) {
                double putLeftY = ramp(currdriveyinput,getLeftY.getAsDouble(),drive_ramprate);
                double putLeftX = ramp(currdrivexinput,getLeftX.getAsDouble(),drive_ramprate);
                //double putRightX = ramp(currrotinput,getRightX.getAsDouble(),turn_ramprate);
                currdriveyinput = putLeftY;
                currdrivexinput = putLeftX;
                //currrotinput = putRightX;
                drive.setControl(aim2
                        .withVelocityX(putLeftY * MaxSpeed)
                        .withVelocityY(putLeftX * MaxSpeed)
                        .withTargetDirection(gethubdirection()));
            } else {
                currdriveyinput = 0;
                currdrivexinput = 0;
                currrotinput = 0;
                drive.setControl(aim2.withVelocityX(0).withVelocityY(0).withTargetDirection(gethubdirection()));
            }
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }

    /** True once the drivetrain's heading is within 3 degrees of the hub direction. */
    public boolean isrobotaligned() {
        Rotation2d targetHeading = aim2.TargetDirection;
        Rotation2d currentHeadingBlueAlliance = drive.getState().Pose.getRotation();
        Rotation2d currentHeading = currentHeadingBlueAlliance.rotateBy(drive.getOperatorForwardDirection());
        return (MathUtil.isNear(MathUtil.angleModulus(targetHeading.getRadians()), MathUtil.angleModulus(currentHeading.getRadians()),Degrees.of(3).in(Radians),-Math.PI,Math.PI));
    }
    /** Field-relative heading from the robot's current pose to the hub. */
    private Rotation2d gethubdirection() {
        /* if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            hub_xposition = hub_xposition_blue;
        } else {
            hub_xposition = hub_xposition_red;
        }
 */
        hub_xposition = hubpositioninfo.hubPosition().getX();
     //   SmartDashboard.putNumber("hub_xposition", hub_xposition);
     //   SmartDashboard.putNumber("angleflip",hubpositioninfo.cameraflip());


        final Translation2d hub_position = hubpositioninfo.hubPosition();

        Translation2d robotPosition = drive.getState().Pose.getTranslation();
        // robotPosition.addVisionMeasurement(17,4);
        Rotation2d hubdirectionBlueAlliance = hub_position.minus(robotPosition).getAngle();
        //SmartDashboard.putNumber("HubDirectionBlueAllianceAngle",hubdirectionBlueAlliance.getRadians());
        double angle = Math.atan2(hub_position.getY() - robotPosition.getY(),hub_position.getX() - robotPosition.getX());
        //SmartDashboard.putNumber("angle",angle);
        // if (hub_xposition == hub_xposition_blue){
        //     cameraflip = Math.PI;
        // } else {
        //     cameraflip = 0;
        // }
        angle = hubpositioninfo.cameraflip()+Math.atan2(hub_position.getY() - robotPosition.getY(),hub_position.getX() - robotPosition.getX());
        //SmartDashboard.putNumber("revised angle", angle);

        return new Rotation2d(angle);
        //Rotation2d hubdirection = hubdirectionBlueAlliance.rotateBy(m_drivetrain.getOperatorForwardDirection());
        // return hubdirectionBlueAlliance.rotateBy(drive.getOperatorForwardDirection() + 180);
    }

    /** Moves currentValue toward targetValue by at most rampRate per call. */
    public static double ramp(double currentValue, double targetValue, double rampRate){
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

    }
}
