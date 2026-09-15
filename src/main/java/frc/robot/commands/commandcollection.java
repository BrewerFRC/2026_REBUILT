package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;

/**
 * Bundles the drivetrain, intake, feeder, shooter, hood, and hopper
 * subsystems into higher-level composite commands (e.g. auto-aim-and-shoot)
 * that need to coordinate more than one subsystem at a time.
 */
public final class commandcollection {
    private final CommandSwerveDrivetrain drive;
    private final intakesubsystem intake;
    private final feedersubsystem feeder;
    private final shootersubsystem shooter;
    private final hoodsubsystem hood;
    private final hoppersubsystem hopper;

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;

    public commandcollection(
            CommandSwerveDrivetrain drive,
            intakesubsystem intake,
            feedersubsystem feeder,
            shootersubsystem shooter,
            hoodsubsystem hood,
            hoppersubsystem hopper,
            DoubleSupplier forwardInput,
            DoubleSupplier leftInput
    ) {
        this.drive = drive;
        this.intake = intake;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.hopper = hopper;

        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
    }

    public commandcollection(
            CommandSwerveDrivetrain drive,
            intakesubsystem intake,
            feedersubsystem feeder,
            shootersubsystem shooter,
            hoodsubsystem hood,
            hoppersubsystem hopper
    ) {
        this(
                drive,
                intake,
                feeder,
                shooter,
                hood,
                hopper,
                () -> 0,
                () -> 0
        );
    }

    /*
    public Command AimAndShootCommand() {
        final gethubAimDistanceCommand gethubdirection = new gethubAimDistanceCommand(drive, forwardInput.getAsDouble(),leftInput.getAsDouble());
        final ShooterRevCommand ShooterRevCommand = new ShooterRevCommand(shooter);
        return Commands.parallel(
                gethubdirection,
                Commands.waitSeconds(0.25)
                        .andThen(ShooterRevCommand)
                //Commands.waitUntil(() -> gethubdirection.isrobotaligned() && ShooterRevCommand.isReadyToShoot())
                //        .andThen(feeder())
        );//get feeder too working in this
    }
        */

        /**
         * Turns the drivetrain to face the hub while revving the shooter, then
         * once both are ready, feeds a game piece through hood/feeder/hopper.
         */
        public Command AimAndShootCommand() {
                final gethubAimDistanceCommand gethubdirection = new gethubAimDistanceCommand(drive, 0,0);
                final ShooterRevCommand ShooterRevCommand = new ShooterRevCommand(shooter);
                final FeederPassiveReverseCommand feederPassiveReverseCommand = new FeederPassiveReverseCommand(feeder);
                //final HoodOnCommand hoodoncommand = new HoodOnCommand(hood);
                //final FeederOnCommand feed = new FeederOnCommand(feeder);
                //final HopperOnCommand hopperoncommand = new HopperOnCommand(hopper);
                return Commands.parallel(
                        gethubdirection,
                        Commands.waitSeconds(0.25)
                                .andThen(ShooterRevCommand), //alongWith(feederPassiveReverseCommand)
                        Commands.waitUntil(() -> gethubdirection.robotaligned() && ShooterRevCommand.isReadyToShoot())
                                .andThen(hoodfeederhopper())
                        //         .andThen(feed)
                        //         .andThen(hopperoncommand)
                        // Commands.waitUntil(()-> gethubdirection.robotaligned() && ShooterRevCommand.isReadyToShoot())
                        //         .alongWith(hoodoncommand),
                        // Commands.waitSeconds(0.5)                     //waitUntil(()-> hoodoncommand.isHoodReady())
                        //         .alongWith(feed).alongWith(hopperoncommand)

                );

                // return Commands.sequence(
                //         Commands.parallel(
                //                 gethubdirection
                //                 ShooterRevCommand
                //                 hoodoncommand
                //         ),
                //         Commands.waitUntil(() -> gethubdirection.robotaligned() && ShooterRevCommand.isReadyToShoot() && hoodoncommand.isHoodReady())
                //         .andThen(null),
                // );
        }

        /** Raises the hood, then briefly after feeds a game piece through the hopper. */
        private Command hoodfeederhopper() {
                final FeederOnCommand feederon = new FeederOnCommand(feeder);
                // final FeederOffCommand feederoff = new FeederOffCommand(feeder);
                 final HopperOnCommand hopperoncommand = new HopperOnCommand(hopper);
                // final HopperOffCommand hopperoffcommand = new HopperOffCommand(hopper);
                 final HoodOnCommand hoodon = new HoodOnCommand(hood);
                // final HoodOffCommand hoodoffcommand = new HoodOffCommand(hood);
                return Commands.parallel(hoodon,
                        Commands.waitSeconds(0.5).andThen(feederon).alongWith(hopperoncommand)
                        //Commands.waitUntil(()->hoodon.isHoodReady()).andThen(feederon).alongWith((Commands.waitSeconds(0.25).andThen(hopperoncommand)))

                        //Commands.waitSeconds(0.25).andThen(feederon).alongWith(hopperoncommand)


                        // Commands.waitUntil(()->hoodon.isHoodReady()).andThen(feederon).alongWith(hopperoncommand)
                       // Commands.parallel(feederon,hopperoncommand)
                // return Commands.sequence(
                //         Commands.waitSeconds(0.25),
                //         Commands.parallel(feederon, hoodon,
                //                 Commands.waitUntil(()->hoodon.isHoodReady())
                //                         .andThen(hopperoncommand))
                        //Commands.startEnd(() -> hood.hoodOn(), ()->hood.hoodOff(),hood),
                        //Commands.startEnd(() -> feeder.feederOn(), ()->feeder.feederOff(),feeder),
                        //Commands.startEnd(() -> hopper.hopperOn(), ()->hopper.hopperOff(),hopper)
                );
        }


        }