# 2026 REBUILT Robot Code

CTRE swerve robot code for FIRST Robotics Competition's 2026 game, REBUILT.

## Build and deploy

This is a Java 17 GradleRIO project configured for WPILib 2026.2.1 and Team 4564. It includes the vendor dependencies used by the source: CTRE Phoenix 6, REVLib, PathPlanner, and WPILib New Commands.

Install the 2026 WPILib tools, then from the project root run `./gradlew.bat build` to compile and test, or `./gradlew.bat deploy` to build and deploy to the configured team's roboRIO. The Gradle wrapper is committed, so a separate Gradle install is not required. In VS Code, open this repository and use the WPILib commands to build, test, deploy, or manage vendor libraries.

See [the project layout](docs/project-layout.md) for the ownership of source, deploy assets, tests, vendor dependencies, and development tools.

New students should start with the [command-based lifecycle](docs/command-based-lifecycle.md), then use the [control map](docs/controls.md), [REBUILT match flow](docs/match-flow.md), and [code-review checklist](docs/student-code-review-checklist.md) while making changes.

## REBUILT behavior implemented here

- The scoring HUB is active during AUTO, the 10-second transition, and the final 30-second endgame.
- During the four 25-second alliance shifts, the FMS `R`/`B` game-data character selects the alliance whose HUB is inactive first. HUB status is operator feedback only: the driver can still intentionally fire FUEL into an inactive HUB to transport it around the field.
- The dashboard publishes `REBUILT Hub Active`, `REBUILT Hub Phase`, and `REBUILT Game Data` so operators can verify the FMS data before SHIFT 1.
- Hub aiming uses the 2026 manual's nominal field center and 158.6-inch hub distance from the alliance wall, rather than the prior season's coordinates.

The Driver Station's practice-match configuration must match the 2026 timing: 20 seconds AUTO, 110 seconds TELEOP, and 30 seconds endgame. The FMS match time is approximate; drivers should always treat field lighting as the authority.

See [the 2026 REBUILT Game Manual](https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf) (§§5.2, 5.4, 6.4.1) and [WPILib's 2026 game-data reference](https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html) before changing shift logic.
