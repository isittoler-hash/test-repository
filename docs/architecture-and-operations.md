# Architecture and Operations Guide

This document is the "how it works" reference for the codebase. It describes runtime flow, subsystem responsibilities, and the expected robot behavior during both autonomous and driver control.

## 1. Runtime lifecycle

The VEX competition runtime enters the code through `main()` in `src/main.cpp`.

1. `Competition.autonomous(autonomous)` registers the autonomous callback.
2. `Competition.drivercontrol(usercontrol)` registers the driver callback.
3. `pre_auton()` initializes devices using `vexcodeInit()`.
4. VEX competition state machine calls either `autonomous()` or `usercontrol()`.

## 2. Module map

### `include/robot-config.h` and `src/robot-config.cpp`
- Declares and defines all global VEX device objects.
- Centralizes motor port mapping and inversion.
- Applies startup stopping modes:
  - Drive motors: `brake`
  - Conveyor motors: `coast`

### `include/subsystems.h` and `src/subsystems.cpp`
Contains two major subsystem layers:

1. **ConveyorController**
   - Implements named modes (`Collect`, `ScoreHigh`, `ScoreMiddle`, `ReversePurge`, `Off`).
   - Applies motor directions/speeds per mode.
   - Adds anti-jam behavior using bottom-motor velocity/current heuristics.

2. **Wheel-encoder motion primitives**
   - `driveDistanceMm(...)`: proportional control to target wheel rotation from linear mm input.
   - `turnRobotDeg(...)`: converts robot yaw request into wheel arc distance based on track width.
   - `collectWhileDriving(...)`: combined driving + intake mode with anti-jam updates.
   - `holdDrive(...)`: brief hold to settle robot before scoring.

### `src/autonomous_skills.cpp`
- Encodes the autonomous route as a sequence of reusable primitives.
- Uses repeated collect/turn/drive/score cycles.
- Ends with a park maneuver.

### `src/main.cpp`
- Driver mode maps joystick and buttons to drive + conveyor actions.
- Runs anti-jam logic continuously while conveyor modes are active.

## 3. Control strategy details

### 3.1 Drive distance

`driveDistanceMm` uses:
- Distance error in wheel degrees.
- Proportional term (`kDriveKp`) to generate speed.
- Cross-side synchronization correction (`kSyncKp`) from left/right encoder delta.
- Min-output floor (`kMinDrivePct`) to overcome static friction.
- Timeout guard to avoid stalls.

### 3.2 In-place turn

`turnRobotDeg` computes arc length:

`arc = pi * track_width * |deg| / 360`

Then maps arc to wheel degrees and runs a proportional loop with timeout.

### 3.3 Anti-jam logic

Anti-jam is active in forward-feed modes (not off/reverse/middle-score).
A jam condition is inferred when:
- Bottom intake velocity is below threshold, and
- Bottom intake current is above threshold,
- for longer than `kJamDetectMs`.

When jammed:
- Conveyor enters `ReversePurge` briefly (`kUnjamReverseMs`), then
- returns to the previous commanded mode.

## 4. Driver controls

Default mappings in `usercontrol()`:
- Tank drive: `Axis3` (left), `Axis2` (right)
- `L1`: `Collect`
- `L2`: `ReversePurge`
- `R1`: `ScoreHigh`
- `R2`: `ScoreMiddle`
- No button: `Off`

## 5. Tuning workflow

Tune in this order:
1. `kWheelDiameterMm`
2. `kTrackWidthMm`
3. `kDriveKp`, `kTurnKp`, `kSyncKp`
4. `kMinDrivePct`, `kMinTurnPct`
5. Anti-jam thresholds/timers
6. Per-step autonomous distances and angles

## 6. Known tradeoffs

- Encoder-only dead reckoning drifts with wheel slip and contact events.
- No heading reset feedback (inertial disabled by design constraints).
- Route should be treated as a baseline template and field-trimmed.
