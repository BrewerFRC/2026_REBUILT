# Project Layout

| Location | Purpose |
| --- | --- |
| `src/main/java/frc/robot` | Deployable robot code, organized by commands, subsystems, generated Tuner code, and utilities. |
| `src/main/deploy` | Files copied unchanged to `/home/lvuser/deploy` on the roboRIO; PathPlanner assets belong here. |
| `src/test/java` | Desktop JUnit tests for logic that can run without robot hardware. |
| `vendordeps` | Checked-in third-party library manifests used by GradleRIO. |
| `.wpilib` | WPILib project metadata, including the deploy team number. |
| `tools` | Code-generation and maintenance artifacts that are neither compiled nor deployed. |
| `gradlew*`, `gradle/wrapper`, `build.gradle`, `settings.gradle` | The committed GradleRIO build entry points and configuration. |

Build products, Gradle caches, logs, and simulator window state are ignored. Keep deploy assets, vendor manifests, generated source that is required to compile, and wrapper files under version control.
