#include "subsystems.h"

using namespace vex;

controller Controller1;

void runAutonomousSkills();

void pre_auton() {
  vexcodeInit();
}

void autonomous() {
  runAutonomousSkills();
}

void usercontrol() {
  ConveyorController conveyor;

  while (true) {
    const int left = Controller1.Axis3.position(pct);
    const int right = Controller1.Axis2.position(pct);

    FrontLeft.spin(fwd, left, pct);
    BackLeft.spin(fwd, left, pct);
    FrontRight.spin(fwd, right, pct);
    BackRight.spin(fwd, right, pct);

    // L1: collect/up-chain, L2: full reverse purge.
    if (Controller1.ButtonL1.pressing()) {
      conveyor.setMode(ConveyorMode::Collect, 100.0);
    } else if (Controller1.ButtonL2.pressing()) {
      conveyor.setMode(ConveyorMode::ReversePurge, 90.0);
    } else if (Controller1.ButtonR1.pressing()) {
      // Score high: push chain upward.
      conveyor.setMode(ConveyorMode::ScoreHigh, 100.0);
    } else if (Controller1.ButtonR2.pressing()) {
      // Score middle: run top downward while lower chain feeds.
      conveyor.setMode(ConveyorMode::ScoreMiddle, 100.0);
    } else {
      conveyor.stop();
    }

    conveyor.updateAntiJam();
    task::sleep(20);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    task::sleep(100);
  }
}
