#include <cmath>
#include "subsystems.h"

using namespace vex;

namespace {
// ---- Physical + control constants -------------------------------------------------
// Wheel diameter and effective track width are the most important calibration values
// for encoder-only dead reckoning. Re-tune these first when distances/turns drift.
constexpr double kWheelDiameterMm = 82.55;   // 3.25" omni estimate; tune on field.
constexpr double kTrackWidthMm = 285.0;      // center-to-center wheel track; tune on robot.

// Minimum command floors to overcome static friction at low errors.
constexpr double kMinDrivePct = 14.0;
constexpr double kMinTurnPct = 12.0;

// Anti-jam heuristic thresholds.
constexpr double kJamVelocityThresholdRpm = 10.0;
constexpr double kJamCurrentThresholdAmp = 2.0;
constexpr int kJamDetectMs = 260;
constexpr int kUnjamReverseMs = 220;

// Proportional gains for distance/turn and side-to-side synchronization.
constexpr double kDriveKp = 0.12;
constexpr double kTurnKp = 0.24;
constexpr double kSyncKp = 0.08;

// Utility clamp for scalar values.
double clamp(double v, double lo, double hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

// Returns +1 for non-negative values and -1 for negative values.
double sign(double v) {
  return (v >= 0.0) ? 1.0 : -1.0;
}

// Commands both motors on each drive side in percent output.
void setDrivePct(double left, double right) {
  FrontLeft.spin(fwd, left, pct);
  BackLeft.spin(fwd, left, pct);
  FrontRight.spin(fwd, right, pct);
  BackRight.spin(fwd, right, pct);
}

// Stops all drive motors with the provided stopping mode.
void stopDrive(brakeType mode = brakeType::brake) {
  FrontLeft.stop(mode);
  BackLeft.stop(mode);
  FrontRight.stop(mode);
  BackRight.stop(mode);
}

// Converts linear travel in mm to wheel rotation in motor degrees.
double mmToWheelDeg(double mm) {
  const double wheelCircMm = kWheelDiameterMm * 3.14159265358979323846;
  return (mm / wheelCircMm) * 360.0;
}
}  // namespace

void ConveyorController::applyMode(ConveyorMode mode, double pct_cmd) {
  const double pct_out = clamp(pct_cmd, 0.0, 100.0);

  // Each mode controls the full conveyor stack as a coordinated unit.
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
      // For this robot, high-goal scoring uses the same directional flow as collect.
      BottomIntake.spin(fwd, pct_out, pct);
      MiddleIntake.spin(fwd, pct_out, pct);
      TopIntake1.spin(fwd, pct_out, pct);
      TopIntake2.spin(fwd, pct_out, pct);
      break;

    case ConveyorMode::ScoreMiddle:
      // Middle-goal release keeps the lower stages feeding upward while reversing the
      // top stage to route game objects into the middle exit path.
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
  const double clampedPct = clamp(pct, 0.0, 100.0);

  // Skip re-apply if effective command is unchanged.
  if (mode == mode_ && fabs(clampedPct - cmdPct_) < 0.01) {
    return;
  }

  mode_ = mode;
  cmdPct_ = clampedPct;

  // Reset anti-jam state whenever the operator or autonomous changes mode.
  antiJamActive_ = false;
  jamTimer_.clear();
  unjamTimer_.clear();

  applyMode(mode_, cmdPct_);
}

void ConveyorController::stop() {
  setMode(ConveyorMode::Off, 0.0);
}

void ConveyorController::updateAntiJam() {
  // Anti-jam is intentionally disabled in off/reverse/middle modes.
  if (mode_ == ConveyorMode::Off || mode_ == ConveyorMode::ReversePurge ||
      mode_ == ConveyorMode::ScoreMiddle) {
    return;
  }

  const double bottomRpm = fabs(BottomIntake.velocity(rpm));
  const double bottomCurrent = BottomIntake.current(currentUnits::amp);

  if (!antiJamActive_) {
    // Jam signature: low velocity + high current sustained for threshold duration.
    if (bottomRpm < kJamVelocityThresholdRpm && bottomCurrent > kJamCurrentThresholdAmp) {
      if (jamTimer_.time(msec) > kJamDetectMs) {
        antiJamActive_ = true;
        unjamTimer_.clear();

        // Brief reverse pulse to clear the obstruction.
        applyMode(ConveyorMode::ReversePurge, 58.0);
      }
    } else {
      // Condition recovered; restart detection timer.
      jamTimer_.clear();
    }
  } else if (unjamTimer_.time(msec) > kUnjamReverseMs) {
    antiJamActive_ = false;
    jamTimer_.clear();

    // Resume original user/autonomous command.
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

    // Position error in wheel degrees.
    const double error = targetDeg - avgPos;
    if (fabs(error) < 8.0) {
      break;
    }

    // Synchronization correction: positive means left is ahead of right.
    const double sync = (leftPos - rightPos) * kSyncKp;

    // P-only speed control with saturation and minimum-output floor.
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

  // Convert desired robot heading change into wheel arc distance per side.
  const double arcMm = (3.14159265358979323846 * kTrackWidthMm) * (fabs(deg) / 360.0);
  const double targetWheelDeg = mmToWheelDeg(arcMm);
  const double dir = sign(deg);

  timer t;
  while (t.time(msec) < timeout_ms) {
    // Turning loop uses absolute wheel travel from both sides.
    const double leftPos = fabs(leftDriveDeg());
    const double rightPos = fabs(rightDriveDeg());
    const double avg = (leftPos + rightPos) * 0.5;

    const double error = targetWheelDeg - avg;
    if (error < 7.0) {
      break;
    }

    double cmd = error * kTurnKp;
    cmd = clamp(cmd, kMinTurnPct, max_pct);

    // Opposing signs create in-place turn.
    setDrivePct(dir * cmd, -dir * cmd);
    task::sleep(10);
  }

  stopDrive();
}

void holdDrive(int hold_ms) {
  // Short hold to reduce coast/roll before scoring action.
  stopDrive(brakeType::hold);
  task::sleep(hold_ms);
  stopDrive(brakeType::brake);
}

void collectWhileDriving(ConveyorController& conveyor, double mm, double speed_pct) {
  conveyor.setMode(ConveyorMode::Collect, 95.0);

  resetDriveEncoders();
  const double targetDeg = mmToWheelDeg(mm);
  timer t;

  // Time-bounded integrated move: intake while driving to next pickup line.
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

    // Keep jam protection active during heavy intake phases.
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
