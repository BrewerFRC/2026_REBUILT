package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/** Default command that keeps the intake stopped whenever no active command owns it. */
public final class IntakeOffCommand extends Command {
  private final IntakeSubsystem intake;

  public IntakeOffCommand(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.intakeOff();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
