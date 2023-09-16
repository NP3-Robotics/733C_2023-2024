
#include "main.h"
#include <cmath>
#include <iostream>

#include "my_stuff/port_declerations.h"
#include "my_stuff/global_var.h"

using namespace std;

void controllerFunc(pros::Controller controller, double &speed, int &catapos, bool &wingState)
{
    bool speedUp = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP);
    bool slowDown = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN);

    bool goForward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    bool goBackward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

    double turn = pow(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127, 3);

    bool wingButton = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1);

    bool cataButton = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A);
    bool cataArmButton = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B);

    bool intakeIn = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    bool intakeOut = controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);

    bool changeDirection = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X);

    bool runPIDTest = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1);

    if (speedUp && speed < 1)
    {
        speed += 0.1;
    }
    if (slowDown && speed > 0.2)
    {
        speed -= 0.1;
    }
    if (changeDirection)
        speed *= -1;

    if (goForward)
    {
        leftMtrs.move(127 * speed);
        rightMtrs.move(127 * speed);
        std::vector<double> avgLeftMtr = leftMtrs.get_positions();
        std::cout << "Mtr 1 " << avgLeftMtr[0] << " Mtr 2 " << avgLeftMtr[1] << " Mtr 3 " << avgLeftMtr[2] << endl;
        std::vector<double> avgRightMtr = rightMtrs.get_positions();
        std::cout << "Mtr 1 " << avgRightMtr[0] << " Mtr 2 " << avgRightMtr[1] << " Mtr 3 " << avgRightMtr[2] << endl;
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

    if (turn != 0)
    {
        leftMtrs.move(-127 * turn);
        rightMtrs.move(127 * turn);
    }

    if (wingButton)
    {
        if (wingState)
        {
            wingLeft.set_value(true);
            wingRight.set_value(true);
        }
        else
        {
            wingLeft.set_value(false);
            wingRight.set_value(false);
        }

        wingState = !wingState;
    }

    if (cataButton)
    {
        // 11 per shot
        // 7 shaved gear per shot
        // 110 deg for shot, 180 for next
        catapult.move_relative(110, 100);
        pros::delay(500);
        catapult.move_relative(70, 100);
        cataArm.move_relative(-10, 200);
        if (!cataArmMove)
            cataArmMove = !cataArmMove;
    }
    if (cataArmButton && cataArmMove)
    {
        cataArm.move_relative(10, 200);
        cataArmMove = !cataArmMove;
    }

    if (intakeIn)
        intake.move(127);
    else if (intakeOut)
        intake.move(-127);
    else
        intake.move(0);

    if (runPIDTest)
    {
        autonomous();
        pros::delay(10000);
    }
}

void opcontrol()
{
    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    // partner controller
    // pros::Controller controlPartner(pros::E_CONTROLLER_PARTNER);

    double speed = 1;
    int catapos = 0;

    while (true)
    {
        pros::lcd::set_text(0, std::to_string(speed));

        controllerFunc(controller, speed, catapos, wingState);

        pros::delay(10);
    }
}
