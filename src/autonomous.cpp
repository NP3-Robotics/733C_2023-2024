#include "main.h"
#include <iostream>

#include "my_stuff/port_declerations.h"
// #include "my_stuff/lvgl_stuff.h"

#define PI 3.141592653589793238462643383279502884L
using namespace std;

// meant to store things which are needed in PID loop
class PID_Holder
{
public:
    bool left;
    bool moving;

    double startInput = 0;
    double lastInput = 0;
    double output = 0;

    double error = 0;
    double errorAcc = 0;
};

// gives average encoder value of a side
double avgEncoder(std::vector<double> mtr)
{
    std::cout << mtr[0] << " " << mtr[1] << " " << mtr[2] << endl;
    double avgMtrEncoder =
        (mtr[0] + mtr[1] + mtr[2]) / 3;

    return avgMtrEncoder;
}

// converts degree to a distance
double convertDegToDist(double degree)
{
    // 2 pi r
    // r = 2
    // single encoder value = 2 pi r deg / 360 = pi deg / 90

    return PI * degree / 90;
}

// 55, 0, 0 starts oscillating
double kpM = 55;
double kiM = 1;
double kdM = -100;

// finds the new output in the PID loop
PID_Holder findPIDOutput(double desiredPoint, PID_Holder &input)
{
    // what will be returned to the motor
    PID_Holder tempStorage;

    double mtrPos;

    // finding out which motor to check
    if (input.left)
        mtrPos = avgEncoder(leftMtrs.get_positions());
    else
        mtrPos = avgEncoder(rightMtrs.get_positions());

    // seeing if it is the start of the PID loop
    if (!(input.moving))
    {
        tempStorage.lastInput = 0;
        tempStorage.startInput = mtrPos;
        tempStorage.errorAcc = 0;

        tempStorage.moving = true;
    }

    // calcuation of error
    double changeInEncoder = mtrPos - input.startInput;
    double distTravelled = convertDegToDist(changeInEncoder);
    double error = desiredPoint - distTravelled;

    // i term, error accumulation
    tempStorage.errorAcc = input.error + error;

    // test this later, see if adding ki makes the robot wack
    // meant to limit error accumulation
    if (tempStorage.errorAcc > desiredPoint)
        tempStorage.errorAcc = 0;

    if (tempStorage.error < desiredPoint * 0.1)
        tempStorage.errorAcc = 0;

    // new output
    tempStorage.output = kpM * error + kiM * tempStorage.errorAcc + kdM * (distTravelled - input.lastInput) / 10;

    // limit output within bounds of motor velocity (in our case since green 200)
    if (tempStorage.output > 200)
        tempStorage.output = 200;
    if (tempStorage.output < -200)
        tempStorage.output = -200;

    tempStorage.lastInput = tempStorage.output;

    return tempStorage;
}

// work in progress do to restructuring
void moveDist(double desiredDistance)
{
    PID_Holder leftMtrStorage;
    PID_Holder rightMtrStorage;

    while (true)
    {
        PID_Holder currentLeftMtrStorage = findPIDOutput(desiredDistance, leftMtrStorage);
        PID_Holder currentRightMtrStorage = findPIDOutput(desiredDistance, rightMtrStorage);

        if (!leftMtrStorage.moving && !rightMtrStorage.moving)
        {
            cout << "done moving" << endl;
            break;
        }

        leftMtrs.move_velocity(leftMtrStorage.output);
        rightMtrs.move_velocity(rightMtrStorage.output);

        leftMtrStorage = currentLeftMtrStorage;
        rightMtrStorage = currentRightMtrStorage;

        pros::delay(10);
    }
}

double kpT = 10;
double kiT = 0;
double kdT = 0;

// same concept as the one above except for turning
PID_Holder findPIDOutputTurn(double turnAmount, PID_Holder &mtrStorage)
{
    double newHeading = inertial.get_heading();

    if (!(mtrStorage.moving))
    {
        mtrStorage.startInput = newHeading;
        mtrStorage.moving = true;
    }

    double desiredHeading = mtrStorage.startInput + turnAmount;

    if (turnAmount > 0)
        mtrStorage.error = desiredHeading - newHeading;
    else
        mtrStorage.error = newHeading - desiredHeading;

    mtrStorage.errorAcc += mtrStorage.error;

    if (mtrStorage.errorAcc > desiredHeading)
        mtrStorage.errorAcc = 0;

    if (mtrStorage.error < desiredHeading * 0.1)
        mtrStorage.errorAcc = 0;

    double output = kpT * mtrStorage.error + kiT * mtrStorage.errorAcc + kdT * (newHeading - mtrStorage.lastInput) / 10;

    if (output > 200)
        output = 200;
    if (output < -200)
        output = -200;

    mtrStorage.output = output;
    mtrStorage.lastInput = newHeading;

    return mtrStorage;
}

// same concept as one above except for turning
void turnDistIntertial(double desiredRotation)
{
    // look at get_rotation and get_heading
    // use set_heading and set_rotation in init

    PID_Holder inertialStorage;

    while (true)
    {
        PID_Holder currentInertialStorage = findPIDOutputTurn(desiredRotation, inertialStorage);

        if (!inertialStorage.moving)
        {
            cout << "done turning" << endl;
            break;
        }

        leftMtrs.move_velocity(inertialStorage.output);
        rightMtrs.move_velocity(-inertialStorage.output);

        inertialStorage = currentInertialStorage;

        pros::delay(10);
    }
}

void autonomous()
{
    // makes sure everything is zeroed
    leftMtrs.tare_position();
    rightMtrs.tare_position();

    // planning

    /*
    left side shoot over
         move foward
         turn right 90 (90 deg)
         move backward
         intake out
         move forward
         turn right 90 (90 deg)
         move backward
         intake a triball
         turn left 90 (-90 deg)
         shoot triball over the field
         repeat
         maybe touch the bar
    */
    /*
    right side put in goal
         move foward
         turn left 90 (-90 deg)
         move backward
         intake out
         move foward
         turn left 90 (-90 deg)
         move backward
         intake a triball
         move foward
         turn right 90 (90 deg)
         move backward
         intake out
         repeat
         maybe touch the bar
    */

    moveDist(48.0);
    moveDist(-48.0);
    turnDistIntertial(90.0);
}
