#pragma once

#include "main.h"

extern pros::Vision vision;
extern pros::vision_signature_s_t GREEN_TRIBALL;

extern pros::Motor frontLeftMtr;
extern pros::Motor middleLeftMtr;
extern pros::Motor backLeftMtr;
extern pros::Motor_Group leftMtrs;

extern pros::Motor frontRightMtr;
extern pros::Motor middleRightMtr;
extern pros::Motor backRightMtr;
extern pros::Motor_Group rightMtrs;

extern pros::Motor catapult;
extern pros::Motor cataArm;

extern pros::ADIDigitalOut wingLeft;
extern pros::ADIDigitalOut wingRight;

extern pros::Motor intake;