#include <cmath>
#include "subsystems.h"

using namespace vex;

namespace {
constexpr double kWheelDiameterMm = 82.55;   // 3.25" omni estimate; tune on field.
constexpr double kTrackWidthMm = 285.0;      // center-to-center wheel track; tune on robot.
constexpr double kMinDrivePct = 14.0;
constexpr double kMinTurnPct = 12.0;

constexpr double kJamVelocityThresholdRpm = 10.0;
constexpr double kJamCurrentThresholdAmp = 2.0;
constexpr int kJamDetectMs = 260;
constexpr int kUnjamReverseMs = 220;

constexpr double kDriveKp = 0.12;
constexpr double kTurnKp = 0.24;
constexpr double kSyncKp = 0.08;

double clamp(double v, double lo, double hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

double sign(double v) {
  return (v >= 0.0) ? 1.0 : -1.0;
}

void setDrivePct(double left, double right) {
  FrontLeft.spin(fwd, left, pct);
  BackLeft.spin(fwd, left, pct);
  FrontRight.spin(fwd, right, pct);
  BackRight.spin(fwd, right, pct);
}

void stopDrive(brakeType mode = brakeType::brake) {
  FrontLeft.stop(mode);
  BackLeft.stop(mode);
  FrontRight.stop(mode);
  BackRight.stop(mode);
}

double mmToWheelDeg(double mm) {
  const double wheelCircMm = kWheelDiameterMm * 3.14159265358979323846;
  return (mm / wheelCircMm) * 360.0;
}
}  // namespace

void ConveyorController::applyMode(ConveyorMode mode, double pct_cmd) {
  const double pct_out = clamp(pct_cmd, 0.0, 100.0);

  switch (mode) {
    case ConveyorMode::Off:
      BottomIntake.stop(coast);
      MiddleIntake.stop(coast);
      TopIntake1.stop(coast);
      TopIntake2.stop(coast);
      break;

    case ConveyorMode::Collect:
      BottomIntake.spin(fwd, pct_out, pct);
      MiddleIntake.spin(fwd, pct_out, pct);
      TopIntake1.spin(fwd, pct_out, pct);
      TopIntake2.spin(fwd, pct_out, pct);
      break;

    case ConveyorMode::ScoreHigh:
      BottomIntake.spin(fwd, pct_out, pct);
      MiddleIntake.spin(fwd, pct_out, pct);
      TopIntake1.spin(fwd, pct_out, pct);
      TopIntake2.spin(fwd, pct_out, pct);
      break;

    case ConveyorMode::ScoreMiddle:
      // Keep lower chain feeding while top runs downward to middle output lane.
      BottomIntake.spin(fwd, pct_out * 0.75, pct);
      MiddleIntake.spin(fwd, pct_out * 0.75, pct);
      TopIntake1.spin(reverse, pct_out, pct);
      TopIntake2.spin(reverse, pct_out, pct);
      break;

    case ConveyorMode::ReversePurge:
      BottomIntake.spin(reverse, pct_out, pct);
      MiddleIntake.spin(reverse, pct_out, pct);
      TopIntake1.spin(reverse, pct_out, pct);
      TopIntake2.spin(reverse, pct_out, pct);
      break;
  }
}

void ConveyorController::setMode(ConveyorMode mode, double pct) {
  mode_ = mode;
  cmdPct_ = clamp(pct, 0.0, 100.0);
  antiJamActive_ = false;
  jamTimer_.clear();
  unjamTimer_.clear();
  applyMode(mode_, cmdPct_);
}

void ConveyorController::stop() {
  setMode(ConveyorMode::Off, 0.0);
}

void ConveyorController::updateAntiJam() {
  if (mode_ == ConveyorMode::Off || mode_ == ConveyorMode::ReversePurge ||
      mode_ == ConveyorMode::ScoreMiddle) {
    return;
  }

  const double bottomRpm = fabs(BottomIntake.velocity(rpm));
  const double bottomCurrent = BottomIntake.current();

  if (!antiJamActive_) {
    if (bottomRpm < kJamVelocityThresholdRpm && bottomCurrent > kJamCurrentThresholdAmp) {
      if (jamTimer_.time(msec) > kJamDetectMs) {
        antiJamActive_ = true;
        unjamTimer_.clear();
        applyMode(ConveyorMode::ReversePurge, 58.0);
      }
    } else {
      jamTimer_.clear();
    }
  } else if (unjamTimer_.time(msec) > kUnjamReverseMs) {
    antiJamActive_ = false;
    jamTimer_.clear();
    applyMode(mode_, cmdPct_);
  }
}

void resetDriveEncoders() {
  FrontLeft.resetRotation();
  BackLeft.resetRotation();
  FrontRight.resetRotation();
  BackRight.resetRotation();
}

double leftDriveDeg() {
  return (FrontLeft.position(degrees) + BackLeft.position(degrees)) * 0.5;
}

double rightDriveDeg() {
  return (FrontRight.position(degrees) + BackRight.position(degrees)) * 0.5;
}

void driveDistanceMm(double mm, double max_pct, int timeout_ms) {
  resetDriveEncoders();

  timer t;
  const double targetDeg = mmToWheelDeg(mm);

  while (t.time(msec) < timeout_ms) {
    const double leftPos = leftDriveDeg();
    const double rightPos = rightDriveDeg();
    const double avgPos = (leftPos + rightPos) * 0.5;

    const double error = targetDeg - avgPos;
    if (fabs(error) < 8.0) {
      break;
    }

    const double sync = (leftPos - rightPos) * kSyncKp;

    double cmd = error * kDriveKp;
    cmd = clamp(cmd, -max_pct, max_pct);
    if (fabs(cmd) < kMinDrivePct) {
      cmd = kMinDrivePct * sign(cmd);
    }

    setDrivePct(cmd - sync, cmd + sync);
    task::sleep(10);
  }

  stopDrive();
}

void turnRobotDeg(double deg, double max_pct, int timeout_ms) {
  resetDriveEncoders();

  const double arcMm = (3.14159265358979323846 * kTrackWidthMm) * (fabs(deg) / 360.0);
  const double targetWheelDeg = mmToWheelDeg(arcMm);
  const double dir = sign(deg);

  timer t;
  while (t.time(msec) < timeout_ms) {
    const double leftPos = fabs(leftDriveDeg());
    const double rightPos = fabs(rightDriveDeg());
    const double avg = (leftPos + rightPos) * 0.5;

    const double error = targetWheelDeg - avg;
    if (error < 7.0) {
      break;
    }

    double cmd = error * kTurnKp;
    cmd = clamp(cmd, kMinTurnPct, max_pct);

    setDrivePct(dir * cmd, -dir * cmd);
    task::sleep(10);
  }

  stopDrive();
}

void holdDrive(int hold_ms) {
  stopDrive(brakeType::hold);
  task::sleep(hold_ms);
  stopDrive(brakeType::brake);
}

void collectWhileDriving(ConveyorController& conveyor, double mm, double speed_pct) {
  conveyor.setMode(ConveyorMode::Collect, 95.0);

  resetDriveEncoders();
  const double targetDeg = mmToWheelDeg(mm);
  timer t;

  while (t.time(msec) < 2600) {
    const double leftPos = leftDriveDeg();
    const double rightPos = rightDriveDeg();
    const double avgPos = (leftPos + rightPos) * 0.5;

    const double error = targetDeg - avgPos;
    if (fabs(error) < 8.0) {
      break;
    }

    const double sync = (leftPos - rightPos) * kSyncKp;
    double cmd = clamp(error * kDriveKp, -speed_pct, speed_pct);
    if (fabs(cmd) < kMinDrivePct) {
      cmd = kMinDrivePct * sign(cmd);
    }

    setDrivePct(cmd - sync, cmd + sync);
    conveyor.updateAntiJam();
    task::sleep(10);
  }

  stopDrive();
}

void scoreHighGoalCycle(ConveyorController& conveyor, int settle_ms) {
  holdDrive(120);
  conveyor.setMode(ConveyorMode::ScoreHigh, 100.0);
  task::sleep(settle_ms);
}

void scoreMiddleGoalCycle(ConveyorController& conveyor, int settle_ms) {
  holdDrive(120);
  conveyor.setMode(ConveyorMode::ScoreMiddle, 100.0);
  task::sleep(settle_ms);
}
