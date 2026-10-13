# REBUILT match flow

```text
AUTO (20 s) --> TRANSITION (10 s) --> SHIFT 1 --> SHIFT 2 --> SHIFT 3 --> SHIFT 4 --> ENDGAME (30 s)
 HUB active          HUB active        25 s       25 s       25 s       25 s       HUB active
                                      ^ alternates according to FMS R/B game data ^
```

`RebuiltMatchState` turns FMS game data and match time into dashboard feedback: `REBUILT Hub Active`, `REBUILT Hub Phase`, and `REBUILT Game Data`. If game data or alliance is unavailable, it reports a conservative inactive state during shifts; field lights and FMS are the authority.

The status is informational rather than a motor interlock. A driver can still use `AimAndShoot` or the regular shooting sequence to transport FUEL through an inactive HUB when that is the desired strategy.
