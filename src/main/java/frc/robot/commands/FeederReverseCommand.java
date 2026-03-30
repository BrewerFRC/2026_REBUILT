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
//import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.feedersubsystem;
//import frc.robot.subsystems.feedersubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FeederReverseCommand extends Command {


  feedersubsystem m_feeder;
  
  /** Creates a new shooterOnCommand. */
  public FeederReverseCommand(feedersubsystem param_feeder) {
    m_feeder = param_feeder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_feeder);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_feeder.init();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_feeder.feederReverse();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
