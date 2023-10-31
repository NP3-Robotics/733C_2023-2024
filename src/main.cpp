#include "main.h"

#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/optical.h"
#include "pros/rtos.hpp"
#include <iostream>

#include "my_stuff/port_declerations.h"
#include "my_stuff/global_var.h"
// #include "my_stuff/lvgl_stuff.h"

using namespace std;

bool wingState;

void initBot()
{
  cout << "init bot" << endl;
  while (!inertial.is_calibrating())
    pros::delay(2);
  inertial.set_heading(0);

  leftMtrs.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
  leftMtrs.tare_position();
  rightMtrs.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
  rightMtrs.tare_position();

  wingLeft.set_value(false);
  wingRight.set_value(false);
  wingState = false;

  catapult.tare_position();
  catapult.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  // catapult.move_relative(136, 40);

  // while (!(catapult.get_position() < 141 && catapult.get_position() > 131))
  // {
  //   cout << catapult.get_temperature() << endl;
  //   pros::delay(2);
  // }

  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  pros::delay(1000);
  cout << "done init" << endl;
}

void initialize()
{
  initBot();
}

void disabled() {}

void competition_initialize()
{
}