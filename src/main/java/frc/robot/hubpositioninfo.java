package frc.robot;
import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Field-relative info about the scoring hub's position, mirrored across the
 * two alliance-specific sides of the field.
 */
public class hubpositioninfo {
        /** Returns the hub's fixed field position for whichever alliance we're on. */
        public static Translation2d hubPosition() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return new Translation2d(Inches.of(182.105), Inches.of(158.845));
        }
        return new Translation2d(Inches.of(469.115), Inches.of(158.845));
        }

        /**
         * Returns the heading offset (radians) needed to account for the field
         * being mirrored between alliances: 180 degrees on Blue, none on Red.
         */
        public static double cameraflip() {
            final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return Math.PI;
        }
        return 0;
        }
}
