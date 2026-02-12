#include "subsystems.h"

using namespace vex;

// 1-minute autonomous route, wheel-feedback only.
// Start assumption: red side, facing centerline.
void runAutonomousSkills() {
  ConveyorController conveyor;

  // --- Cycle 1: collect near red balls -> score high side goal ---
  collectWhileDriving(conveyor, 900, 68);
  turnRobotDeg(42, 50);
  driveDistanceMm(360, 42);
  scoreHighGoalCycle(conveyor, 520);

  // --- Cycle 2: pull out, collect center cluster -> score middle goal (top runs downward) ---
  driveDistanceMm(-250, 50);
  turnRobotDeg(-82, 55);
  collectWhileDriving(conveyor, 980, 70);
  turnRobotDeg(-36, 50);
  driveDistanceMm(310, 40);
  scoreMiddleGoalCycle(conveyor, 520);

  // --- Cycle 3: opposite center pile -> high goal again ---
  driveDistanceMm(-280, 55);
  turnRobotDeg(96, 55);
  collectWhileDriving(conveyor, 1120, 72);
  turnRobotDeg(38, 50);
  driveDistanceMm(340, 42);
  scoreHighGoalCycle(conveyor, 580);

  // --- Cycle 4: quick middle recycle ---
  driveDistanceMm(-220, 55);
  turnRobotDeg(-68, 55);
  collectWhileDriving(conveyor, 760, 68);
  turnRobotDeg(-34, 48);
  driveDistanceMm(260, 38);
  scoreMiddleGoalCycle(conveyor, 520);

  // --- Endgame: park in red parking zone ---
  conveyor.stop();
  driveDistanceMm(-480, 65);
  turnRobotDeg(128, 60);
  driveDistanceMm(1080, 78);
  holdDrive(300);
}
