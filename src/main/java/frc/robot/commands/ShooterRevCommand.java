// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import com.ctre.phoenix6.hardware.TalonFX;

//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants;
//import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.shootersubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterRevCommand extends Command {

  shootersubsystem m_rev;
  double RPS;
  /** Creates a new shooterOnCommand. */
  public ShooterRevCommand(shootersubsystem param_rev) {
    m_rev = param_rev;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_rev);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_rev.RevShooters();
    m_rev.InterpolatedShoot();
    //m_rev.ShooterReady();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_rev.ShooterReady();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean isReadyToShoot() {
        return m_rev.ShooterReady();
    }

}
