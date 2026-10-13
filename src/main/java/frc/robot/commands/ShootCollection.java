package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** Builds the non-aiming shoot sequence used by the driver's trigger and autonomous events. */
public final class ShootCollection {
  private static final double FEED_DELAY_SECONDS = 0.50;

  private final FeederSubsystem feeder;
  private final ShooterSubsystem shooter;
  private final HopperSubsystem hopper;
  private final HoodSubsystem hood;

  public ShootCollection(
      FeederSubsystem feeder, ShooterSubsystem shooter, HopperSubsystem hopper, HoodSubsystem hood) {
    this.feeder = feeder;
    this.shooter = shooter;
    this.hopper = hopper;
    this.hood = hood;
  }

  /** Revs the shooter and starts feeding after the measured shooter speed is ready. */
  public Command shootCommand() {
    ShooterRevCommand shooterRev = new ShooterRevCommand(shooter);
    return Commands.parallel(
        shooterRev,
        Commands.waitUntil(shooterRev::isReadyToShoot).andThen(feedFuel()));
  }

  private Command feedFuel() {
    return Commands.parallel(
        new HoodOnCommand(hood),
        Commands.waitSeconds(FEED_DELAY_SECONDS)
            .andThen(new FeederOnCommand(feeder))
            .alongWith(new HopperOnCommand(hopper)));
  }
}
