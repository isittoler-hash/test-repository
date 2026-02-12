#include "robot-config.h"

using namespace vex;

// Global VEX brain and competition objects.
brain Brain;
competition Competition;

// Drive train motor mapping.
// Port + reverse states are defined to match current robot wiring.
motor FrontLeft(PORT12, ratio18_1, false);
motor BackLeft(PORT11, ratio18_1, false);
motor FrontRight(PORT16, ratio18_1, true);
motor BackRight(PORT19, ratio18_1, true);

// Conveyor/intake chain motor mapping.
motor BottomIntake(PORT20, ratio18_1, false);
motor MiddleIntake(PORT1, ratio18_1, false);
motor TopIntake1(PORT5, ratio18_1, false);
motor TopIntake2(PORT2, ratio18_1, true);

void vexcodeInit() {
  // Drive train uses brake mode for better positional settling between commands.
  FrontLeft.setStopping(brakeType::brake);
  BackLeft.setStopping(brakeType::brake);
  FrontRight.setStopping(brakeType::brake);
  BackRight.setStopping(brakeType::brake);

  // Conveyor chain coasts by default to avoid abrupt load spikes when released.
  BottomIntake.setStopping(brakeType::coast);
  MiddleIntake.setStopping(brakeType::coast);
  TopIntake1.setStopping(brakeType::coast);
  TopIntake2.setStopping(brakeType::coast);
}
