#pragma once

#include "robot-config.h"

enum class ConveyorMode {
  Off,
  Collect,      // ground pickup and internal indexing
  ScoreHigh,    // move balls upward to high release
  ScoreMiddle,  // run top downward for middle-goal release
  ReversePurge  // clear jams
};

class ConveyorController {
 public:
  void setMode(ConveyorMode mode, double pct = 100.0);
  void stop();
  void updateAntiJam();

 private:
  bool antiJamActive_ = false;
  vex::timer jamTimer_;
  vex::timer unjamTimer_;
  ConveyorMode mode_ = ConveyorMode::Off;
  double cmdPct_ = 100.0;

  void applyMode(ConveyorMode mode, double pct);
};

void resetDriveEncoders();
double leftDriveDeg();
double rightDriveDeg();

// Wheel-encoder-only drivetrain primitives.
void driveDistanceMm(double mm, double max_pct = 70.0, int timeout_ms = 2500);
void turnRobotDeg(double deg, double max_pct = 55.0, int timeout_ms = 1800);
void holdDrive(int hold_ms = 120);

void collectWhileDriving(ConveyorController& conveyor, double mm, double speed_pct = 65.0);
void scoreHighGoalCycle(ConveyorController& conveyor, int settle_ms = 450);
void scoreMiddleGoalCycle(ConveyorController& conveyor, int settle_ms = 450);
