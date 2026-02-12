# API Reference (Team-Facing)

This is a quick reference for the public interfaces in `include/subsystems.h` and `include/robot-config.h`.

## Global devices (`robot-config.h`)

- `vex::brain Brain`
- `vex::competition Competition`
- Drive motors: `FrontLeft`, `BackLeft`, `FrontRight`, `BackRight`
- Conveyor motors: `BottomIntake`, `MiddleIntake`, `TopIntake1`, `TopIntake2`
- `void vexcodeInit()`

## Conveyor modes

```cpp
enum class ConveyorMode {
  Off,
  Collect,
  ScoreHigh,
  ScoreMiddle,
  ReversePurge
};
```

- `Off`: stop all conveyor motors (`coast`).
- `Collect`: all stages forward for intake/indexing.
- `ScoreHigh`: all stages forward to release upward.
- `ScoreMiddle`: lower stages forward, top stages reversed.
- `ReversePurge`: all stages reversed to clear jams.

## `ConveyorController`

### `void setMode(ConveyorMode mode, double pct = 100.0)`
Sets active conveyor mode and command percent (0 to 100). Resets anti-jam internal timers/state and applies mode immediately.

### `void stop()`
Shortcut for `setMode(ConveyorMode::Off, 0.0)`.

### `void updateAntiJam()`
Checks bottom intake telemetry and performs temporary purge reversal when jam conditions persist.
Call this periodically (every loop tick) while the conveyor is active.

## Drive helper functions

### `void resetDriveEncoders()`
Zeros all four drive motor encoders.

### `double leftDriveDeg()` / `double rightDriveDeg()`
Returns average encoder position (degrees) for each side of tank drive.

## Motion primitives

### `void driveDistanceMm(double mm, double max_pct = 70.0, int timeout_ms = 2500)`
Closed-loop linear move using wheel encoders.
- Positive `mm`: forward
- Negative `mm`: reverse

### `void turnRobotDeg(double deg, double max_pct = 55.0, int timeout_ms = 1800)`
Closed-loop in-place turn from desired robot yaw angle.
- Positive `deg`: one turn direction
- Negative `deg`: opposite direction

### `void holdDrive(int hold_ms = 120)`
Applies hold braking briefly to settle robot pose before returning to brake mode.

## Composite autonomous helpers

### `void collectWhileDriving(ConveyorController& conveyor, double mm, double speed_pct = 65.0)`
Runs intake collect mode while driving a target distance.

### `void scoreHighGoalCycle(ConveyorController& conveyor, int settle_ms = 450)`
Settles robot and runs high-goal scoring mode.

### `void scoreMiddleGoalCycle(ConveyorController& conveyor, int settle_ms = 450)`
Settles robot and runs middle-goal scoring mode.
