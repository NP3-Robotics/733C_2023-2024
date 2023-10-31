#include "main.h"
#include <cmath>
#include <iostream>

#include "my_stuff/port_declerations.h"
#include "my_stuff/global_var.h"
// #include "my_stuff/lvgl_stuff.h"

using namespace std;

void controllerFunc(pros::Controller controller, double &speed, int &catapos, bool &wingState)
{
    bool cataPosition = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP);
    bool robotHeading = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN);
    bool changeDirection = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1);

    bool goForward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    bool goBackward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

    double turn = pow(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127, 3);
    bool turnSmall = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

    bool wingButton = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1);

    bool cataButton = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A);
    bool manual = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);

    bool intakeIn = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    bool intakeOut = controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);

    bool distanceTravelled = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X);

    bool resetCata = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y);

    if (changeDirection)
        speed *= -1;

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

    if (turn != 0)
    {
        leftMtrs.move(127 * turn);
        rightMtrs.move(-127 * turn);
    }

    if (turnSmall != 0)
    {
        leftMtrs.move(50 * controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)));
        rightMtrs.move(-50 * controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)));
        // cout << "Turn input " << turnSmall << endl;
        // cout << "Controller input " << controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) << endl;
    }

    if (wingButton)
    {
        if (wingState)
        {
            wingLeft.set_value(false);
            wingRight.set_value(false);
        }
        else
        {

            wingLeft.set_value(true);
            wingRight.set_value(true);
        }

        wingState = !wingState;
    }

    if (cataButton)
    {
        double finalPos = catapult.get_position() + 185;

        catapult.move_absolute(finalPos, 35);

        int t = 0;

        while (!(catapult.get_position() < finalPos + 5 && catapult.get_position() > finalPos - 5))
        {
            cout << catapult.get_position() << endl;
            t += 2;
            if (t > 5000)
                break;
            pros::delay(2);
        }
    }
    else if (manual)
    {
        catapult.move_velocity(35);
    }
    else
        catapult.move_velocity(0);

    if (resetCata)
    {
        double finalPos = 135;
        if ((catapult.get_position() - 135) / 180 <= abs(0.5))
        {
            while (finalPos < catapult.get_position())
            {
                finalPos += 180;
            }
        }
        catapult.move_absolute(finalPos, 35);

        while (!(catapult.get_position() < finalPos + 5 && catapult.get_position() > finalPos - 5))
        {
            cout << catapult.get_position() << endl;
            pros::delay(2);
        }
    }

    if (intakeIn)
        intake.move(127);
    else if (intakeOut)
        intake.move(-127);
    else
        intake.move(0);
}

void opcontrol()
{
    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    double speed = 1;
    int catapos = 0;
    double lastHeading = 0;

    while (true)
    {
        controllerFunc(controller, speed, catapos, wingState);

        cout << "Temperature is " << catapult.get_temperature() << endl;

        pros::delay(10);
    }
}
