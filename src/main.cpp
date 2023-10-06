#include "main.h"

#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/optical.h"
#include "pros/rtos.hpp"
#include "stdio.h"

#include "my_stuff/port_declerations.h"
#include "my_stuff/global_var.h"
// #include "my_stuff/lvgl_stuff.h"

using namespace std;

pros::Vision vision(1);

pros::Motor frontLeftMtr(1, pros::E_MOTOR_GEAR_GREEN, false,
                         pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor middleLeftMtr(2, pros::E_MOTOR_GEAR_GREEN, false,
                          pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor backLeftMtr(3, pros::E_MOTOR_GEAR_GREEN, true,
                        pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group leftMtrs({frontLeftMtr, middleLeftMtr, backLeftMtr});

pros::Motor frontRightMtr(8, pros::E_MOTOR_GEAR_GREEN, true,
                          pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor middleRightMtr(9, pros::E_MOTOR_GEAR_GREEN, true,
                           pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor backRightMtr(10, pros::E_MOTOR_GEAR_GREEN, false,
                         pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group rightMtrs({frontRightMtr, middleRightMtr, backRightMtr});

pros::Motor catapult(21, pros::E_MOTOR_GEAR_RED, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::ADIDigitalOut wingLeft('A');
pros::ADIDigitalOut wingRight('B');

pros::Motor intake(7, pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::IMU inertial(6);

bool wingState;
bool cataArmMove;

void initBot()
{
  if (!inertial.is_calibrating())
    pros::lcd::set_text(1, "Done calibrating inertial sensor");

  leftMtrs.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
  leftMtrs.tare_position();
  rightMtrs.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
  rightMtrs.tare_position();

  wingLeft.set_value(false);
  wingRight.set_value(false);
  wingState = false;

  // vision.set_zero_point(pros::E_VISION_ZERO_CENTER);

  catapult.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  catapult.move_relative(135, 100);
  cataArmMove = true;
}

void initialize()
{
  pros::lcd::initialize();
  initBot();
}

void disabled() {}

void competition_initialize()
{
}