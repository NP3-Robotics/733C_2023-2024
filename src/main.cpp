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

bool wingState;
bool cataArmMove;
double cataPos = 0;

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

  // vision.set_zero_point(pros::E_VISION_ZERO_CENTER);

  catapult.tare_position();
  catapult.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  cataPos = 0;

  // catapult.move_absolute(137, 40);

  // while (!(catapult.get_position() < 146 && catapult.get_position() > 136))
  // {
  //   cout << catapult.get_position() << endl;
  //   pros::delay(2);
  // }

  // cout << catapult.get_position() - cataPos << endl;

  catapult.move_relative(137, 40);

  cataPos += catapult.get_position();
  cataArmMove = true;

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