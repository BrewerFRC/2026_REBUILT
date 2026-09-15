package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;

/**
 * Composes the shooter, hood, feeder, and hopper subsystems into the
 * full "spin up then feed" shooting sequence used by driver controls
 * and autonomous named commands.
 */
public final class ShootCollection {
    private final feedersubsystem feeder;
    private final shootersubsystem shooter;
    private final hoppersubsystem hopper;
    private final hoodsubsystem hood;


    public ShootCollection(
            feedersubsystem feeder,
            shootersubsystem shooter,
            hoppersubsystem hopper,
            hoodsubsystem hood
    ) {
        this.feeder = feeder;
        this.shooter = shooter;
        this.hopper = hopper;
        this.hood = hood;

    }


        /** Revs the shooter, then once it's at speed runs the hood/feeder/hopper. */
        public Command ShootCommand() {
                final ShooterRevCommand ShooterRevCommand = new ShooterRevCommand(shooter);
                // final FeederPassiveReverseCommand feederPassiveReverseCommand = new FeederPassiveReverseCommand(feeder);
                // final FeederOnCommand feed = new FeederOnCommand(feeder);
                // final HopperOnCommand hopperoncommand = new HopperOnCommand(hopper);
                return Commands.parallel(
                        ShooterRevCommand,
                        // feederPassiveReverseCommand,

                        Commands.waitUntil(() -> ShooterRevCommand.isReadyToShoot())
                                .andThen(hoodfeederhopper())
                );

        }

        /** Raises the hood, then briefly after feeds a game piece through the hopper. */
        private Command hoodfeederhopper() {
                final FeederOnCommand feederon = new FeederOnCommand(feeder);
                // final FeederOffCommand feederoff = new FeederOffCommand(feeder);
                 final HopperOnCommand hopperoncommand = new HopperOnCommand(hopper);
                // final HopperOffCommand hopperoffcommand = new HopperOffCommand(hopper);
                        final HoodOnCommand hoodoncommand = new HoodOnCommand(hood);

                return Commands.parallel(hoodoncommand,
                        Commands.waitSeconds(0.5).andThen(feederon).alongWith(hopperoncommand)
                
                );
        }


        }