#include "robot-config.h"

using namespace vex;

brain Brain;
competition Competition;

// Port + reverse states from your screenshots.
motor FrontLeft(PORT12, ratio18_1, false);
motor BackLeft(PORT11, ratio18_1, false);
motor FrontRight(PORT16, ratio18_1, true);
motor BackRight(PORT19, ratio18_1, true);

motor BottomIntake(PORT20, ratio18_1, false);
motor MiddleIntake(PORT1, ratio18_1, false);
motor TopIntake1(PORT5, ratio18_1, false);
motor TopIntake2(PORT2, ratio18_1, true);

void vexcodeInit() {
  FrontLeft.setStopping(brakeType::brake);
  BackLeft.setStopping(brakeType::brake);
  FrontRight.setStopping(brakeType::brake);
  BackRight.setStopping(brakeType::brake);

  BottomIntake.setStopping(brakeType::coast);
  MiddleIntake.setStopping(brakeType::coast);
  TopIntake1.setStopping(brakeType::coast);
  TopIntake2.setStopping(brakeType::coast);
}
