#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/optical.h"
#include "pros/rtos.hpp"
#include "stdio.h"

/*
#include "my_stuff/init.h"
#include "my_stuff/auton.h"
#include "my_stuff/op_control.h"
*/

#include "my_stuff/port_declerations.h"
#include "my_stuff/global_var.h"

using namespace std;

pros::Vision vision(1);

pros::Motor frontLeftMtr(1, pros::E_MOTOR_GEAR_GREEN, false,
                         pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor middleLeftMtr(2, pros::E_MOTOR_GEAR_GREEN, true,
                          pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor backLeftMtr(3, pros::E_MOTOR_GEAR_GREEN, false,
                        pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group leftMtrs({frontLeftMtr, middleLeftMtr, backLeftMtr});

pros::Motor frontRightMtr(4, pros::E_MOTOR_GEAR_GREEN, true,
                          pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor middleRightMtr(5, pros::E_MOTOR_GEAR_GREEN, false,
                           pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor backRightMtr(6, pros::E_MOTOR_GEAR_GREEN, true,
                         pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group rightMtrs({frontRightMtr, middleRightMtr, backRightMtr});

pros::Motor catapult(7, pros::E_MOTOR_GEAR_RED, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor cataArm(8, pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::ADIDigitalOut wingLeft('A');
pros::ADIDigitalOut wingRight('B');

pros::Motor intake(9, pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);

bool wingState;
bool cataArmMove;

void initBot()
{
  leftMtrs.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
  leftMtrs.tare_position();
  rightMtrs.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
  rightMtrs.tare_position();

  wingLeft.set_value(true);
  wingRight.set_value(true);
  wingState = false;

  // vision.set_zero_point(pros::E_VISION_ZERO_CENTER);

  catapult.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  catapult.move_relative(135, 100);
  cataArm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  cataArm.move_relative(10, 200);
  cataArmMove = true;
  pros::delay(500);
}

void initialize()
{
  pros::lcd::initialize();
  initBot();
}

void disabled() {}

// void controllerFunc(int leftY, int rightX)
// {
//   if (leftY != 0 && rightX == 0)
//   {
//     leftMtrs.move(127 * leftY);
//     rightMtrs.move(127 * leftY);
//   }

//   if (leftY == 0 && rightX != 0)
//   {
//     leftMtrs.move(127 * rightX);
//     rightMtrs.move(127 * -rightX);
//   }

//   if (leftY != 0 && rightX != 0)
//   {
//     cout << "im here" << endl;
//   }
// }
