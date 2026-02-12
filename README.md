# VEX V5 autonomous codebase (wheel-feedback only)

This project targets a VEX V5 robot with a tank drive and multi-stage conveyor. It is intentionally designed around **wheel encoder feedback only** for autonomous control (no inertial/vision/distance sensors in-loop).

## Quick architecture summary

The codebase is organized into four layers:

1. **Hardware configuration** (`robot-config`) for motors and stopping modes.
2. **Reusable subsystems** (`subsystems`) for conveyor behavior and wheel-encoder motion primitives.
3. **Autonomous route definition** (`autonomous_skills`) built from subsystem helpers.
4. **Competition entrypoints** (`main`) that bind autonomous/driver callbacks and operator controls.

For a deeper walkthrough, see:
- `docs/architecture-and-operations.md`
- `docs/api-reference.md`
- `docs/current-game-research.md`

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

## Conveyor behavior model

- **Collect / High scoring:** chain runs upward/forward.
- **Middle scoring:** top stage runs downward while lower stages continue feeding.
- **Reverse purge:** all stages reverse to clear blockages.
- **Anti-jam:** monitors bottom intake RPM/current and applies brief automatic reverse pulses.

## Autonomous strategy (high-level)

- Repeated cycle pattern: collect -> align -> approach -> score.
- Alternates high and middle scoring cycles for flexibility.
- Finishes with endgame parking in the red zone.

Because this is encoder-only dead reckoning, field slip/contact will introduce drift. The route is expected to be iteratively tuned.

## VS Code VEX extension compatibility

This repository now includes the common VEX project metadata expected by the VS Code VEX extension:
- `include/vex.h` umbrella header
- `.vscode/vex_project_settings.json` project descriptor

If the extension still does not recognize the project, use the extension command to reopen/import the current folder as a V5 C++ project.

## Files

- `src/robot-config.cpp`: robot ports, inversion, and stopping mode setup.
- `src/subsystems.cpp`: conveyor modes, anti-jam logic, and wheel-only motion control.
- `src/autonomous_skills.cpp`: 1-minute route with collect/score cycles and parking finish.
- `src/main.cpp`: competition hooks + driver controls.
- `docs/architecture-and-operations.md`: detailed runtime and control explanation.
- `docs/api-reference.md`: quick function and interface reference.

## Tuning checklist on field

1. Tune `kWheelDiameterMm` and `kTrackWidthMm` in `src/subsystems.cpp`.
2. Tune `kDriveKp`, `kTurnKp`, `kSyncKp` if approach or turning behavior is unstable.
3. Tune cycle-specific distances/turns in `src/autonomous_skills.cpp`.
4. Tune anti-jam thresholds (`kJamVelocityThresholdRpm`, `kJamCurrentThresholdAmp`) and jam timings.
5. Verify parking endpoint in red zone from the real start tile.
