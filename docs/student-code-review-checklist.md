# Student code-review checklist

- [ ] Build and run all tests with `./gradlew.bat test`.
- [ ] Put wiring IDs, ports, and tunable named constants in `Constants`; do not add unexplained numeric literals.
- [ ] Give each command its required subsystem(s) through `addRequirements`.
- [ ] Define the safe motor state in `end()` for held commands and in the subsystem's default command.
- [ ] Keep `RobotContainer` to wiring, bindings, autonomous selection, and PathPlanner event registration.
- [ ] Use `PascalCase` for class names, `camelCase` for fields/methods, and `UPPER_SNAKE_CASE` for constants.
- [ ] Explain why a non-obvious value or safety decision exists; remove obsolete experiments and commented-out code.
- [ ] Add a hardware-free test for new field math, timing boundaries, or pure helper logic.
- [ ] Update `docs/controls.md` whenever a control binding changes.
- [ ] Test motor direction and limit behavior with the robot safely supported before enabling on the floor.
