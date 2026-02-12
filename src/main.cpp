#include "subsystems.h"

using namespace vex;

// Competition controller used only in driver mode.
controller Controller1;

// Implemented in src/autonomous_skills.cpp.
void runAutonomousSkills();

void pre_auton() {
  // Ensure all motors have the expected stopping modes and hardware init is complete
  // before entering either autonomous or driver control.
  vexcodeInit();
}

void autonomous() {
  // Delegate autonomous behavior to the route file so main stays clean.
  runAutonomousSkills();
}

void usercontrol() {
  ConveyorController conveyor;

  while (true) {
    // Standard tank drive mapping:
    // - Axis3 controls left side
    // - Axis2 controls right side
    const int left = Controller1.Axis3.position(pct);
    const int right = Controller1.Axis2.position(pct);

    FrontLeft.spin(fwd, left, pct);
    BackLeft.spin(fwd, left, pct);
    FrontRight.spin(fwd, right, pct);
    BackRight.spin(fwd, right, pct);

    // Conveyor mode priority is first-match in this chain.
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

    // Keep anti-jam active during user control for any forward-feed mode.
    conveyor.updateAntiJam();
    task::sleep(20);
  }
}

int main() {
  // Register callbacks with VEX competition framework.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  // Keep process alive forever; VEX runtime invokes callbacks as needed.
  while (true) {
    task::sleep(100);
  }
}
