#include "main.h"
#include <iostream>

#include "my_stuff/port_declerations.h"
// #include "my_stuff/lvgl_stuff.h"

#define PI 3.141592653589793238462643383279502884L
using namespace std;

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

double avgEncoder(std::vector<double> mtr)
{
    // Responsible for finding position of the motors
    double avgMtrEncoder =
        (mtr[0] + mtr[1] + mtr[2]) / 3;

    return avgMtrEncoder;
}

double convertDegToDist(double degree)
{
    // Used to determine distance travlled

    // Circumfrance 2 pi r
    // Radius of the wheel = 2
    // Distance travelled per degree  = 2 pi r deg / 360 = pi deg / 90

    return PI * degree / 90;
}

// 55, 0, 0 starts oscillating
double kpM = 25;
double kiM = 0;
double kdM = 0;

PID_Holder findPIDOutput(double desiredPoint, PID_Holder input)
{
    // what will be returned to the motor
    PID_Holder tempStorage = input;

    double mtrPos;

    // finding out which motor to check for current position
    if (input.left)
        mtrPos = avgEncoder(leftMtrs.get_positions());
    else
        mtrPos = avgEncoder(rightMtrs.get_positions());

    // seeing if it is the start of the PID loop
    if (!input.moving)
    {
        tempStorage.startInput = mtrPos;

        tempStorage.moving = true;
    }

    // calcuation of error
    double changeInEncoder = mtrPos - input.startInput;
    double distTravelled = convertDegToDist(changeInEncoder);
    tempStorage.error = desiredPoint - distTravelled;

    // i term, error accumulation
    tempStorage.errorAcc = input.error + tempStorage.error;

    // test this later, see if adding ki makes the robot wack
    // meant to limit error accumulation
    if (tempStorage.errorAcc > desiredPoint)
        tempStorage.errorAcc = 0;

    if (tempStorage.error < desiredPoint * 0.3)
        tempStorage.errorAcc = 0;

    // new output
    tempStorage.output = kpM * tempStorage.error + kiM * tempStorage.errorAcc + kdM * (distTravelled - input.lastInput);

    // limit output within bounds of motor velocity (in our case since green 200)
    if (tempStorage.output > 200)
        tempStorage.output = 200;
    if (tempStorage.output < -200)
        tempStorage.output = -200;

    // setting lastInput
    tempStorage.lastInput = distTravelled;

    return tempStorage;
}

void moveDist(double desiredDistance)
{
    PID_Holder leftMtrStorage;
    PID_Holder rightMtrStorage;

    leftMtrStorage.left = true;
    rightMtrStorage.left = false;

    leftMtrs.tare_position();
    rightMtrs.tare_position();

    while (true)
    {
        leftMtrStorage = findPIDOutput(desiredDistance, leftMtrStorage);
        rightMtrStorage = findPIDOutput(desiredDistance, rightMtrStorage);

        cout << leftMtrStorage.startInput << endl;

        leftMtrs.move_velocity(leftMtrStorage.output);
        rightMtrs.move_velocity(rightMtrStorage.output);

        if (abs(leftMtrStorage.output) < 1 && abs(rightMtrStorage.output) < 1)
            break;

        pros::delay(15);
    }
    leftMtrs.move_velocity(0);
    rightMtrs.move_velocity(0);
}

double kpT = 2;
double kiT = 0;
double kdT = 0;

class PID_Holder_Turn
{
public:
    bool moving;

    bool turnLeft;

    double startInput = 0;
    double lastInput = 0;
    double output = 0;

    double desiredPoint1;
    double desiredPoint2;

    double error = 0;
    double errorAcc = 0;
};

PID_Holder_Turn findPIDOutputTurn(double turnAmount, PID_Holder_Turn input)
{
    // what will be returned to the motor
    PID_Holder_Turn tempStorage = input;

    if (turnAmount < 0)
        tempStorage.turnLeft = true;
    else
        tempStorage.turnLeft = false;

    // finding current position
    double currentHeading = inertial.get_heading();

    if (currentHeading == 360)
        currentHeading = 0;

    // seeing if it is the start of the PID loop
    if (!(tempStorage.moving))
    {
        tempStorage.startInput = currentHeading;
        tempStorage.moving = true;
    }

    // finding end point
    tempStorage.desiredPoint1 = tempStorage.startInput + turnAmount;

    // makes sure everything is in terms of 360 deg
    if (tempStorage.desiredPoint1 < 360)
        tempStorage.desiredPoint2 = tempStorage.desiredPoint1 + 360;
    else
        tempStorage.desiredPoint2 = tempStorage.desiredPoint1 - 360;

    double desiredPoint;

    if (abs(currentHeading - tempStorage.desiredPoint1) < abs(currentHeading - tempStorage.desiredPoint2))
        desiredPoint = tempStorage.desiredPoint1;
    else
        desiredPoint = tempStorage.desiredPoint2;

    tempStorage.error = desiredPoint - currentHeading;

    // i term, error accumulation
    tempStorage.errorAcc += tempStorage.error;

    // meant to limit error accumulation
    if (tempStorage.errorAcc > tempStorage.desiredPoint1)
        tempStorage.errorAcc = 0;

    if (tempStorage.error < tempStorage.desiredPoint1 * 0.3)
        tempStorage.errorAcc = 0;

    // new output
    tempStorage.output = kpT * tempStorage.error + kiT * tempStorage.errorAcc + kdT * (currentHeading - input.lastInput);

    // limit output within bounds of motor velocity (in our case since green 200)
    if (tempStorage.output > 200)
        tempStorage.output = 200;
    if (tempStorage.output < -200)
        tempStorage.output = -200;

    // setting lastInput
    tempStorage.lastInput = currentHeading;

    return tempStorage;
}

void turnDist(double desiredRotation)
{
    PID_Holder_Turn inertialStorage;

    while (true)
    {
        inertialStorage = findPIDOutputTurn(desiredRotation, inertialStorage);

        if (abs(inertialStorage.output) < 1)
            break;

        leftMtrs.move_velocity(inertialStorage.output);
        rightMtrs.move_velocity(-inertialStorage.output);

        pros::delay(15);
    }

    leftMtrs.move_velocity(0);
    rightMtrs.move_velocity(0);
}

void autonomous()
{
    // initTeamImg();

    leftMtrs.tare_position();
    rightMtrs.tare_position();
    inertial.tare_heading();

    // left side
    //      put matchload in goal
    //      move foward
    //      turn right 90 (90 deg)
    //      move backward
    //      intake out

    //      grab a new triball
    //      move forward
    //      turn right 90 (90 deg)
    //      move backward
    //      intake a triball

    //      send new triball over the field
    //      turn left 90 (-90 deg)
    //      shoot triball over the field
    //      repeat

    //      maybe touch the bar

    // right side put
    //      put matchload in goal
    //      move foward
    //      turn left 90 (-90 deg)
    //      move backward
    //      intake out

    //      grab a triball
    //      move foward
    //      turn left 90 (-90 deg)`
    //      move backward
    //      intake a triball

    //      put new triball in goal
    //      move foward
    //      turn right 90 (90 deg)
    //      move backward
    //      intake out
    //      repeat

    //      maybe touch the bar`

    // 33.9411255 in hypot

    // left
    moveDist(-18.0);
    turnDist(30.0);
    moveDist(-4.0);
    intake.move_velocity(-200);
    pros::delay(1000);
    intake.move_velocity(0);
    moveDist(-1.75);
    moveDist(6.25);
    turnDist(-25.0);
    wingRight.set_value(true);
    moveDist(8.75);
    turnDist(-20.0);
    moveDist(2.0);
    pros::delay(500);
    moveDist(2.0);
    turnDist(-80.0);
    moveDist(25.5);
    turnDist(75.0);
    moveDist(25.0);
    turnDist(-35.0);

    // wingRight.set_value(false);
    // turnDist(-10.0);
    // moveDist(30.0);
    // wingRight.set_value(true);
    // catapult.move_relative(136, 100);
    // turnDist(-5.0);
    // moveDist(17.0);

    // right
    /*
    moveDist(-40.0);
    turnDist(90.0);
    intake.move_velocity(-200);
    pros::delay(750);
    intake.move_velocity(0);
    moveDist(-2);
    */

    // skills
    /*
    int numShots = 0;
    while (numShots < 36)
    {
        double finalPos = catapult.get_position() + 185;

        catapult.move_absolute(finalPos, 35);

        int t = 0;

        while (!(catapult.get_position() < finalPos + 5 && catapult.get_position() > finalPos - 5))
        {
            cout << catapult.get_position() << endl;
            pros::delay(2);
            t += 2;
            if (t > 5000)
                break;
            pros::delay(2);
        }

        numShots++;
    }
    // turnDist(-90.0);
    // moveDist(24);
    */
}