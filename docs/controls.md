# 2026 driver controls

All controls below reflect `RobotContainer.java`. Update this page in the same change as a binding.

| Controller | Control | Action |
| --- | --- | --- |
| Driver | Left stick | Field-centric translation |
| Driver | Right stick X | Field-centric rotation |
| Driver | Y | Reset field-centric heading |
| Driver | POV up/down | Slow robot-centric forward/reverse creep |
| Driver | Right trigger | Rev shooter, then feed FUEL when ready |
| Driver | A | Drive with translation while automatically facing the HUB |
| Driver | B | Aim at HUB, rev shooter, and feed when aligned/ready |
| Driver | X | Drive toward the calibrated HUB shooting distance |
| Operator | Left/right bumper | Lower/raise arm |
| Operator | Left trigger | Run ground intake |
| Operator | Right trigger | Reverse intake, hopper, and feeder to clear a jam |
| Operator | A | Raise/run hood while held |
| Operator | X | Reverse hood while held |

The shooter commands are intentionally **not** blocked when the team's HUB is inactive. During REBUILT, that preserves the strategic transport shot option. Drivers remain responsible for using the current HUB state and field signals.
