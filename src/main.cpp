#include "main.h"
#include "okapi/api.hpp"
#include <iostream>

using namespace std;

using namespace okapi;

Motor left1(1, false, okapi::AbstractMotor::gearset::green,
            okapi::AbstractMotor::encoderUnits::rotations);
Motor left2(2, true, okapi::AbstractMotor::gearset::green,
            okapi::AbstractMotor::encoderUnits::rotations);
Motor left3(3, true, okapi::AbstractMotor::gearset::green,
            okapi::AbstractMotor::encoderUnits::rotations);

Motor right1(4, false, okapi::AbstractMotor::gearset::green,
             okapi::AbstractMotor::encoderUnits::rotations);
Motor right2(5, true, okapi::AbstractMotor::gearset::green,
             okapi::AbstractMotor::encoderUnits::rotations);
Motor right3(6, false, okapi::AbstractMotor::gearset::green,
             okapi::AbstractMotor::encoderUnits::rotations);

MotorGroup leftMtrs({left1, left2, left3});
MotorGroup rightMtrs({right1, right2, right3});

Motor cata(7, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::degrees);

pros::ADIDigitalOut wingLeft('A', true);
pros::ADIDigitalOut wingRight('B', true);

std::shared_ptr<okapi::ChassisController> drive =
    okapi::ChassisControllerBuilder()
        .withMotors(leftMtrs, rightMtrs)
        .withDimensions(okapi::AbstractMotor::gearset::green,
                        {{4_in, 11.5_in}, okapi::imev5GreenTPR})
        .build();

void on_center_button() {}

void initialize()
{
  pros::lcd::initialize();
  cata.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
}

void disabled() {}

void autonomous() {}

void opcontrol()
{
  okapi::Controller controller;

  double direction;
  double turn;
  double velocity = 1;

  okapi::ControllerButton wingOn(okapi::ControllerDigital::R1);
  okapi::ControllerButton wingOff(okapi::ControllerDigital::L1);

  while (true)
  {
    direction = 0;
    if (controller.getDigital(okapi::ControllerDigital::up))
    {
      if (velocity != 1)
      {
        velocity += 0.1;
      }
    }
    if (controller.getDigital(okapi::ControllerDigital::down))
    {
      if (velocity != 0)
      {
        velocity -= 0.1;
      }
    }

    if (controller.getDigital(okapi::ControllerDigital::R2))
    {
      direction = velocity;
    }
    if (controller.getDigital(okapi::ControllerDigital::L2))
    {
      direction = -velocity;
    }

    if (controller.getDigital(okapi::ControllerDigital::A))
    {

      while (true)
      {
        if (cata.getPosition() <= 151)
          cata.moveVelocity(100);
        if (cata.getPosition() > 151)
        {
          // cout << "cata shot encoder value at " << cata.getPosition() << endl;
          cata.moveVelocity(0);
          cata.tarePosition();

          break;
        }
      }

      //  150.8 so far
      cata.moveVelocity(25);
    }
    else
    {
      cata.moveVelocity(0);
    }

    if (wingOn.isPressed())
    {
      wingLeft.set_value(false);
      wingRight.set_value(false);
    }

    if (wingOff.isPressed())
    {
      wingLeft.set_value(true);
      wingRight.set_value(true);
    }

    cout << cata.getPosition() << endl;

    turn = controller.getAnalog(okapi::ControllerAnalog::leftX) / 2;

    drive->getModel()->arcade(direction, turn);

    pros::delay(10);
  }
}
