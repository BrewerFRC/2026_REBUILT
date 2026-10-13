package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/** Holds the intake in reverse while the operator clears a jam. */
public final class IntakeReverseCommand extends Command {
  private final IntakeSubsystem intake;

  public IntakeReverseCommand(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.intakeReverse();
  }

  @Override
  public void end(boolean interrupted) {
    intake.intakeOff();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
