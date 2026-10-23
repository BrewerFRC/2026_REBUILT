# Development Tools

This directory contains files used to generate or maintain robot code. Nothing here is compiled or deployed by GradleRIO.

## CTRE Tuner

`ctre/tuner_constants.py` is the Python Tuner X project artifact for the configured swerve drive. It is retained as a hardware-generation reference; the Java runtime uses the generated classes in `src/main/java/frc/robot/generated/`.

When mechanical configuration, CAN IDs, offsets, or control gains change, regenerate the Java Tuner output and review the resulting diff. Do not hand-edit generated Tuner Java classes.
