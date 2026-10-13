# Command-based lifecycle

WPILib's scheduler calls command methods in this order:

```text
button/path event schedules command
        |
        v
 initialize()     once: reset timers, encoders, or command-local state
        |
        v
 execute()        every ~20 ms: update motors or calculations
        |
        +-- isFinished() false --> execute again
        |
        +-- isFinished() true, button released, or conflict --> end(interrupted)
```

Use `addRequirements(subsystem)` in every command constructor. It tells the scheduler which command owns a mechanism; when a conflicting command starts, the scheduler ends the older one. A subsystem default command runs whenever no other command owns that subsystem, so default commands should normally leave motors in a safe state.

`whileTrue(...)` is the usual choice for a held control. It schedules the command while held and cancels it on release. That is why `IntakeOnCommand.end()` explicitly stops the intake. `onTrue(...)` is for a one-time action. Composite commands in `CommandCollection` build a fresh command for each request because command instances retain scheduler state.

Before writing a new command, answer these questions:

1. Which subsystem(s) does it control?
2. What is the safe state if the command is interrupted?
3. Is it held (`whileTrue`), a one-shot (`onTrue`), or part of a timed/conditional sequence?
4. Can the calculation be put in a pure helper and tested without hardware?
