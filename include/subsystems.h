#pragma once

#include "vex.h"

// High-level conveyor operating modes used by both autonomous and driver control.
enum class ConveyorMode {
  Off,
  Collect,      // ground pickup and internal indexing
  ScoreHigh,    // move balls upward to high release
  ScoreMiddle,  // run top downward for middle-goal release
  ReversePurge  // clear jams
};

// Stateful helper that applies conveyor modes and runs anti-jam recovery.
class ConveyorController {
 public:
  // Applies a mode immediately and stores it as the commanded baseline.
  void setMode(ConveyorMode mode, double pct = 100.0);

  // Convenience wrapper for turning conveyor completely off.
  void stop();

  // Should be called in periodic loops to trigger jam detection/recovery.
  void updateAntiJam();

 private:
  bool antiJamActive_ = false;
  vex::timer jamTimer_;
  vex::timer unjamTimer_;
  ConveyorMode mode_ = ConveyorMode::Off;
  double cmdPct_ = 100.0;

  // Low-level actuator command routine behind each mode.
  void applyMode(ConveyorMode mode, double pct);
};

// Drive encoder utilities.
void resetDriveEncoders();
double leftDriveDeg();
double rightDriveDeg();

// Wheel-encoder-only drivetrain primitives.
void driveDistanceMm(double mm, double max_pct = 70.0, int timeout_ms = 2500);
void turnRobotDeg(double deg, double max_pct = 55.0, int timeout_ms = 1800);
void holdDrive(int hold_ms = 120);

// Composite routines for autonomous readability.
void collectWhileDriving(ConveyorController& conveyor, double mm, double speed_pct = 65.0);
void scoreHighGoalCycle(ConveyorController& conveyor, int settle_ms = 450);
void scoreMiddleGoalCycle(ConveyorController& conveyor, int settle_ms = 450);
