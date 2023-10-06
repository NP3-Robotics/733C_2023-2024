#include "main.h"
#include <cmath>
#include <iostream>

#include "my_stuff/port_declerations.h"
#include "my_stuff/global_var.h"
// #include "my_stuff/lvgl_stuff.h"

using namespace std;

// what each button on the controller does
void controllerFunc(pros::Controller controller, double &speed, int &catapos, bool &wingState)
{
    // what the buttons are, may delete
    bool speedUp = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP);
    bool slowDown = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN);

    bool goForward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    bool goBackward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

    double turn = pow(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127, 3);

    bool wingButton = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1);

    bool cataButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);

    bool intakeIn = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    bool intakeOut = controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);

    bool changeDirection = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X);

    bool runPIDTest = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B);

    bool cataPosition = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1);

    // change speed of bot, bounds make sure the bot doesn't go too fast or too slow
    if (speedUp && speed < 1)
    {
        speed += 0.1;
    }
    if (slowDown && speed > 0.2)
    {
        speed -= 0.1;
    }

    // change direction of input (foward to backward or backward to foward)
    if (changeDirection)
        speed *= -1;

    // foward/backward control
    if (goForward)
    {
        leftMtrs.move(127 * speed);
        rightMtrs.move(127 * speed);
    }
    else if (goBackward)
    {
        leftMtrs.move(-127 * speed);
        rightMtrs.move(-127 * speed);
    }
    else
    {
        leftMtrs.move(0);
        rightMtrs.move(0);
    }

    // check for turn input
    if (turn != 0)
    {
        leftMtrs.move(127 * turn);
        rightMtrs.move(-127 * turn);
    }

    // use wing
    if (wingButton)
    {

        if (wingState)
        {
            // if wing open close
            wingLeft.set_value(false);
            wingRight.set_value(false);
        }
        else
        {
            // if wing close open
            wingLeft.set_value(true);
            wingRight.set_value(true);
        }

        // change state of wing so it can retract or extend on next press
        wingState = !wingState;
    }

    // moving catapult
    if (cataButton)
    {
        // 11 per shot
        // 7 shaved gear per shot
        // 110 deg for shot, 180 for next

        catapult.move_relative(180, 40);
        // catapult.move_velocity(40);
    }
    else
        catapult.move(0);

    // moving intake
    if (intakeIn)
        intake.move(127);
    else if (intakeOut)
        intake.move(-127);
    else
        intake.move(0);

    // meant to see how far cata needs to move, haven't tested yet
    if (cataPosition)
    {
        if (cataArmMove)
        {
            catapos = catapult.get_position();
        }
        else
        {
            cout << catapult.get_position() - catapos << endl;
        }
    }

    // for testing if PID is accurate or needs more tuning
    if (runPIDTest)
        autonomous();
}

void opcontrol()
{
    // declaring controller
    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    // variables needed for controllerFunc()
    double speed = 1;
    int catapos = 0;
    double lastHeading = 0;

    // think this was for checking inertial drift
    double firstHeading = inertial.get_heading();

    double totalTimeInOp = 0;
    double totalInertialDif = 0;

    double newInertialReading;
    double inertialDif;

    while (true)
    {
        // seeing if inertial was done calibrating
        if (!inertial.is_calibrating())
        {
            newInertialReading = inertial.get_heading();
            inertialDif = (newInertialReading - lastHeading) / 10;

            totalInertialDif += totalInertialDif - firstHeading;
            lastHeading = newInertialReading;

            totalTimeInOp += 10;

            // cout << "totalInertialDif is " << totalInertialDif << endl;
            // cout << totalInertialDif / totalTimeInOp << endl;
        }

        // check for drift
        pros::lcd::set_text(1, std::to_string(newInertialReading));
        pros::lcd::set_text(2, std::to_string(inertialDif));
        pros::lcd::set_text(3, std::to_string(totalInertialDif));

        // controller functions
        controllerFunc(controller, speed, catapos, wingState);

        pros::delay(10);
    }
}
