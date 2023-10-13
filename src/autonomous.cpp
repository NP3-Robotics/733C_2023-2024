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
    double avgMtrEncoder =
        (mtr[0] + mtr[1] + mtr[2]) / 3;

    return avgMtrEncoder;
}

double convertDegToDist(double degree)
{
    // 2 pi r
    // r = 2
    // single encoder value = 2 pi r deg / 360 = pi deg / 90

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
    if (!(input.moving))
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
    tempStorage.lastInput = tempStorage.output;

    return tempStorage;
}

void moveDist(double desiredDistance)
{
    PID_Holder leftMtrStorage;
    PID_Holder rightMtrStorage;

    leftMtrStorage.left = true;
    rightMtrStorage.left = false;

    while (true)
    {
        leftMtrStorage = findPIDOutput(desiredDistance, leftMtrStorage);
        rightMtrStorage = findPIDOutput(desiredDistance, rightMtrStorage);

        /*
        if (leftMtrStorage.error == 0 && rightMtrStorage.error == 0)
        {
            // when to exit code, should be implemented once tuning works
            break;
        }
        else
        {
            leftMtrs.move_velocity(leftMtrStorage.output);
            rightMtrs.move_velocity(rightMtrStorage.output);
        }
        */

        leftMtrs.move_velocity(leftMtrStorage.output);
        cout << "left error is " << leftMtrStorage.error << " left output is " << leftMtrStorage.output << endl;
        rightMtrs.move_velocity(rightMtrStorage.output);
        cout << "right error is " << rightMtrStorage.error << " right output is " << rightMtrStorage.output << endl;

        pros::delay(15);
    }
}

double *calcCurveDistance(double desiredDistance, double desiredDegree)
{
    const double rw = 103 / 16;

    double r = desiredDistance / desiredDegree - rw;

    double distanceArr[2];

    distanceArr[0] = r * desiredDegree;

    cout << "Left Distance: " << distanceArr[0] << endl;

    distanceArr[1] = (r + 2 * rw) * desiredDegree;

    cout << "Right Distance: " << distanceArr[1] << endl;

    return distanceArr;
}

void moveCurve(double desiredDistance, double desiredDegree)
{
    double desiredDistanceLeft = calcCurveDistance(desiredDistance, desiredDegree)[0];
    double desiredDistanceRight = calcCurveDistance(desiredDistance, desiredDegree)[1];

    PID_Holder leftMtrStorage;
    PID_Holder rightMtrStorage;

    leftMtrStorage.left = true;
    rightMtrStorage.left = false;

    while (true)
    {
        leftMtrStorage = findPIDOutput(desiredDistanceLeft, leftMtrStorage);
        rightMtrStorage = findPIDOutput(desiredDistanceRight, rightMtrStorage);

        // leftMtrs.move_velocity(leftMtrStorage.output);
        cout << "left error is " << leftMtrStorage.error << " left output is " << leftMtrStorage.output << endl;
        // rightMtrs.move_velocity(rightMtrStorage.output);
        cout << "right error is " << rightMtrStorage.error << " right output is " << rightMtrStorage.output << endl;

        pros::delay(15);
    }
}

double kpT = 1;
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

    // finding current position
    double currentHeading = inertial.get_heading();
    cout << "Current heading is: " << currentHeading << endl;

    // seeing if it is the start of the PID loop
    if (!(tempStorage.moving))
    {
        tempStorage.startInput = currentHeading;
        tempStorage.moving = true;
    }

    // finding end point
    tempStorage.desiredPoint1 = tempStorage.startInput + turnAmount;

    if (tempStorage.desiredPoint1 > 360)
        tempStorage.desiredPoint2 = tempStorage.desiredPoint1 - 360;
    if (tempStorage.desiredPoint1 < 0)
        tempStorage.desiredPoint2 = tempStorage.desiredPoint1 + 360;

    cout << "Desired point 1 is: " << tempStorage.desiredPoint1 << endl;

    // makes sure everything is in terms of 360 deg
    if (currentHeading < 180 && tempStorage.desiredPoint1 > 360)
        tempStorage.desiredPoint1 = tempStorage.desiredPoint2;
    if (currentHeading > 180 && tempStorage.desiredPoint1 < 0)
        tempStorage.desiredPoint1 = tempStorage.desiredPoint2;

    // finding shortest direction
    // cout << "error 1: " << desiredHeading - currentHeading << " error 2: " << currentHeading - desiredHeading << endl;
    // if (abs(desiredHeading - currentHeading) < abs(currentHeading - desiredHeading))

    tempStorage.error = tempStorage.desiredPoint1 - currentHeading;

    // i term, error accumulation
    tempStorage.errorAcc += tempStorage.error;

    // test this later, see if adding ki makes the robot wack
    // meant to limit error accumulation
    if (tempStorage.errorAcc > tempStorage.desiredPoint1)
        tempStorage.errorAcc = 0;

    if (tempStorage.error < tempStorage.desiredPoint1 * 0.3)
        tempStorage.errorAcc = 0;

    // cout << "Error is: " << tempStorage.error << ", ErrorAcc is: " << tempStorage.errorAcc << ", Diff is: " << currentHeading - input.lastInput << endl;

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

void turnDistIntertial(double desiredRotation)
{
    // look at get_rotation and get_heading
    // use set_heading and set_rotation in init

    PID_Holder_Turn inertialStorage;

    while (true)
    {
        inertialStorage = findPIDOutputTurn(desiredRotation, inertialStorage);

        cout << "error is " << inertialStorage.error << " output is " << inertialStorage.output << endl;

        leftMtrs.move_velocity(inertialStorage.output);
        rightMtrs.move_velocity(-inertialStorage.output);

        pros::delay(15);
    }
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
    //      turn left 90 (-90 deg)
    //      move backward
    //      intake a triball

    //      put new triball in goal
    //      move foward
    //      turn right 90 (90 deg)
    //      move backward
    //      intake out
    //      repeat

    //      maybe touch the bar

    // left
    // moveDist(48.0);
    turnDistIntertial(-90.0);
    // moveCurve(20, 90);
}
