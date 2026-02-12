#pragma once

#include "vex.h"

extern vex::brain Brain;
extern vex::competition Competition;

// 4-motor drivetrain (from provided screenshots).
extern vex::motor FrontLeft;
extern vex::motor BackLeft;
extern vex::motor FrontRight;
extern vex::motor BackRight;

// Conveyor/intake chain.
extern vex::motor BottomIntake;
extern vex::motor MiddleIntake;
extern vex::motor TopIntake1;
extern vex::motor TopIntake2;

void vexcodeInit();
