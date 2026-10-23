package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Builds commands that coordinate several subsystems.
 *
 * <p>This class deliberately returns a <em>new</em> command each time. Command instances keep
 * scheduler state, so reuse can make a later button press behave like a continuation of an old
 * one.
 */
public final class CommandCollection {
  private static final double SHOOTER_SPINUP_DELAY_SECONDS = 0.25;
  private static final double FEED_DELAY_SECONDS = 0.50;

  private final CommandSwerveDrivetrain drivetrain;
  private final FeederSubsystem feeder;
  private final ShooterSubsystem shooter;
  private final HoodSubsystem hood;
  private final HopperSubsystem hopper;

  public CommandCollection(
      CommandSwerveDrivetrain drivetrain,
      FeederSubsystem feeder,
      ShooterSubsystem shooter,
      HoodSubsystem hood,
      HopperSubsystem hopper) {
    this.drivetrain = drivetrain;
    this.feeder = feeder;
    this.shooter = shooter;
    this.hood = hood;
    this.hopper = hopper;
  }

  /**
   * Faces the HUB and revs the shooter in parallel. Feeding begins only after aiming and shooter
   * speed are ready. HUB activity is intentionally not checked: transport shots into an inactive
   * HUB remain a legal driver option.
   */
  public Command aimAndShootCommand() {
    HubAimCommand hubAim = new HubAimCommand(drivetrain, 0.0, 0.0);
    ShooterRevCommand shooterRev = new ShooterRevCommand(shooter);

    return Commands.parallel(
        hubAim,
        Commands.waitSeconds(SHOOTER_SPINUP_DELAY_SECONDS).andThen(shooterRev),
        Commands.waitUntil(() -> hubAim.isRobotAligned() && shooterRev.isReadyToShoot())
            .andThen(feedFuel()));
  }

  /** Raises the hood, then starts feeder and hopper together after the mechanism has time to move. */
  private Command feedFuel() {
    return Commands.parallel(
        new HoodOnCommand(hood),
        Commands.waitSeconds(FEED_DELAY_SECONDS)
            .andThen(new FeederOnCommand(feeder))
            .alongWith(new HopperOnCommand(hopper)));
  }
}
