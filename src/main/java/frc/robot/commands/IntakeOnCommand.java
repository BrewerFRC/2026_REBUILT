package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/** Holds the intake on while the scheduling button or path event remains active. */
public final class IntakeOnCommand extends Command {
  private final IntakeSubsystem intake;

  public IntakeOnCommand(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.intakeOn();
  }

  @Override
  public void end(boolean interrupted) {
    // A whileTrue binding cancels this command when released, so always leave the motor safe.
    intake.intakeOff();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
