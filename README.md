# VEX V5 autonomous codebase (wheel-feedback only)

This project is configured for your robot and field goals:

- 4-motor tank drivetrain
- 3-stage intake/conveyor chain + dual top motors
- Autonomous uses **only wheel motor encoder feedback** (no inertial, no vision, no distance sensors)
- 1-minute skills-oriented route with repeated collect/score cycles and end parking

## Robot motor mapping (from provided screenshots)

### Drive
- `FrontLeft` -> Port 12, Normal
- `BackLeft` -> Port 11, Normal
- `FrontRight` -> Port 16, Reverse
- `BackRight` -> Port 19, Reverse

### Conveyor / intake chain
- `BottomIntake` -> Port 20, Normal
- `MiddleIntake` -> Port 1, Normal
- `TopIntake1` -> Port 5, Normal
- `TopIntake2` -> Port 2, Reverse

## Important behavior

- **Collect / High scoring:** chain runs upward/forward.
- **Middle scoring:** top stage runs downward while lower stages continue feeding.
- **Anti-jam:** monitors bottom intake RPM/current and auto-reverses briefly if stalled.

## Files

- `src/robot-config.cpp`: robot ports and direction setup
- `src/subsystems.cpp`: wheel-only drive math, turns, conveyor modes, anti-jam
- `src/autonomous_skills.cpp`: 1-minute route with cycles + red zone park
- `src/main.cpp`: competition hooks + driver controls

## Tuning checklist on field

1. Tune `kWheelDiameterMm` and `kTrackWidthMm` in `src/subsystems.cpp`.
2. Tune each cycle distance/turn in `src/autonomous_skills.cpp`.
3. Tune anti-jam thresholds (`kJamVelocityThresholdRpm`, `kJamCurrentThresholdAmp`).
4. Verify parking endpoint in red zone with your real start tile.
