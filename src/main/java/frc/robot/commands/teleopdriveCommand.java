package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
//import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
//import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
//import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.*;

public class teleopdriveCommand extends Command {
    private CommandSwerveDrivetrain drive;
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

    private final SwerveRequest.FieldCentric fieldcentricrequest = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate*0.1) // Add a 15% deadband to maxSpeed
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public teleopdriveCommand(CommandSwerveDrivetrain commandSwerveDrivetrain, DoubleSupplier getLeftY, DoubleSupplier getLeftX, DoubleSupplier getRightX) {
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
        /* double putLeftY = ramp(currdriveyinput,getLeftY.getAsDouble(),drive_ramprate);
        double putLeftX = ramp(currdrivexinput,getLeftX.getAsDouble(),drive_ramprate);
        double putRightX = ramp(currrotinput,getRightX.getAsDouble(),turn_ramprate);
        currdriveyinput = putLeftY;
        currdrivexinput = putLeftX;
        currrotinput = putRightX;
        drive.setControl(fieldcentricrequest
        .withVelocityX(putLeftY * MaxSpeed)
        .withVelocityY(putLeftX * MaxSpeed)
        .withRotationalRate(putRightX * MaxAngularRate)); */
        if (getLeftX.getAsDouble() == 0 && getLeftY.getAsDouble() == 0 && getRightX.getAsDouble() == 0) {
            inputtracker = inputtracker + 1;
        }
        else {
            inputtracker = 0;
        }
        if (inputtracker <= 2) {
            double putLeftY = ramp(currdriveyinput,getLeftY.getAsDouble(),drive_ramprate);
            double putLeftX = ramp(currdrivexinput,getLeftX.getAsDouble(),drive_ramprate);
            double putRightX = ramp(currrotinput,getRightX.getAsDouble(),turn_ramprate);
            currdriveyinput = putLeftY;
            currdrivexinput = putLeftX;
            currrotinput = putRightX;
            drive.setControl(fieldcentricrequest
                    .withVelocityX(putLeftY * MaxSpeed)
                    .withVelocityY(putLeftX * MaxSpeed)
                    .withRotationalRate(putRightX * MaxAngularRate));
        } else {
            currdriveyinput = 0;
            currdrivexinput = 0;
            currrotinput = 0;
            drive.setControl(fieldcentricrequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }

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
