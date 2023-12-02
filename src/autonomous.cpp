#include "main.h"
#include <iostream>

#include "my_stuff/port_declerations.h"
#include "my_stuff/lvgl_stuff.h"

#define PI 3.141592653589793238462643383279502884L
using namespace std;

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

class cata_PID_Holder2
{
public:
    bool moving;

    double startInput = 0;
    double lastInput = 0;
    double output = 0;

    double error = 0;
    double errorAcc = 0;

    double timeSinceLastMovement;
};

cata_PID_Holder2 cataPIDOutput2(double desiredPoint, double maxVelocity, double kp, double ki, double kd, cata_PID_Holder2 input)
{
    // what will be returned to the motor
    cata_PID_Holder2 tempStorage = input;

    double mtrPos = catapult.get_position();

    // seeing if it is the start of the PID loop
    if (!input.moving)
    {
        tempStorage.startInput = mtrPos;

        tempStorage.moving = true;
    }

    if (tempStorage.lastInput == mtrPos)
        tempStorage.timeSinceLastMovement += 15;
    else
        tempStorage.timeSinceLastMovement = 0;

    // calcuation of error
    tempStorage.error = desiredPoint - mtrPos;

    // i term, error accumulation
    tempStorage.errorAcc = input.error + tempStorage.error;

    // test this later, see if adding ki makes the robot wack
    // meant to limit error accumulation
    if (tempStorage.errorAcc > desiredPoint)
        tempStorage.errorAcc = 0;

    if (tempStorage.error < desiredPoint * 0.3)
        tempStorage.errorAcc = 0;

    // new output
    tempStorage.output = kp * tempStorage.error + ki * tempStorage.errorAcc + kd * (mtrPos - input.lastInput);

    if (abs(tempStorage.output) > maxVelocity)
    {
        if (tempStorage.output > 0)
            tempStorage.output = maxVelocity;
        else
            tempStorage.output = -maxVelocity;
    }

    // setting lastInput
    tempStorage.lastInput = mtrPos;

    return tempStorage;
}

void moveCata2(double desiredDistance, double maxVelocity)
{
    cata_PID_Holder2 cataStorage;

    double timeSinceLastMovement = 0;

    while (true)
    {
        cataStorage = cataPIDOutput2(desiredDistance, maxVelocity, 1, 0, 0, cataStorage);

        catapult.move_velocity(cataStorage.output);

        if (abs(cataStorage.error) < 3)
            break;

        if (cataStorage.timeSinceLastMovement >= 450)
            break;

        pros::delay(15);
    }
    catapult.move_velocity(0);
}

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

    double timeSinceLastMovement = 0;
};

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

    if (tempStorage.lastInput == mtrPos)
        tempStorage.timeSinceLastMovement += 15;
    else
        tempStorage.timeSinceLastMovement = 0;

    // seeing if it is the start of the PID loop
    if (!input.moving)
    {
        tempStorage.startInput = mtrPos;

        tempStorage.moving = true;
        // cout << "Encoder starting value is " << tempStorage.startInput << endl;
    }

    // calcuation of error
    double changeInEncoder = mtrPos - input.startInput;
    double distTravelled = convertDegToDist(changeInEncoder);
    tempStorage.error = desiredPoint - distTravelled;

    /*
    cout << "Desired point is " << desiredPoint << endl;

    if (tempStorage.left)
        cout << "Left error is " << tempStorage.error << ", dist travelled is " << distTravelled << endl;
    */

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

    /*
    if (tempStorage.left)
        cout << "Left error is " << tempStorage.error << ", output is " << tempStorage.output << endl;
    // else
    //     cout << "Right error is " << tempStorage.error << ", output is " << tempStorage.output << endl;
    */

    // setting lastInput
    tempStorage.lastInput = distTravelled;

    return tempStorage;
}

PID_Holder findPIDOutput(double desiredPoint, double maxVelocity, PID_Holder input)
{
    // what will be returned to the motor
    PID_Holder tempStorage = input;

    double mtrPos;

    // finding out which motor to check for current position
    if (input.left)
        mtrPos = avgEncoder(leftMtrs.get_positions());
    else
        mtrPos = avgEncoder(rightMtrs.get_positions());

    if (tempStorage.lastInput == mtrPos)
        tempStorage.timeSinceLastMovement += 15;
    else
        tempStorage.timeSinceLastMovement = 0;

    // seeing if it is the start of the PID loop
    if (!input.moving)
    {
        tempStorage.startInput = mtrPos;

        tempStorage.moving = true;
        // cout << "Encoder starting value is " << tempStorage.startInput << endl;
    }

    // calcuation of error
    double changeInEncoder = mtrPos - input.startInput;
    double distTravelled = convertDegToDist(changeInEncoder);
    tempStorage.error = desiredPoint - distTravelled;

    // cout << "Desired point is " << desiredPoint << endl;

    /*
    if (tempStorage.left)
        cout << "Left error is " << tempStorage.error << ", dist travelled is " << distTravelled << endl;
    */

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

    if (abs(tempStorage.output) > maxVelocity)
    {
        if (tempStorage.output > 0)
            tempStorage.output = maxVelocity;
        else
            tempStorage.output = -maxVelocity;
    }

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

        if (abs(leftMtrStorage.error) < 0.5 && abs(rightMtrStorage.error) < 0.5)
            break;
        if (leftMtrStorage.timeSinceLastMovement >= 450 && rightMtrStorage.timeSinceLastMovement >= 450)
            break;

        pros::delay(15);
    }
    leftMtrs.move_velocity(0);
    rightMtrs.move_velocity(0);
}

void moveDist(double desiredDistance, double maxVelocity)
{
    PID_Holder leftMtrStorage;
    PID_Holder rightMtrStorage;

    leftMtrStorage.left = true;
    rightMtrStorage.left = false;

    leftMtrs.tare_position();
    rightMtrs.tare_position();

    while (true)
    {
        leftMtrStorage = findPIDOutput(desiredDistance, maxVelocity, leftMtrStorage);
        rightMtrStorage = findPIDOutput(desiredDistance, maxVelocity, rightMtrStorage);

        cout << leftMtrStorage.startInput << endl;

        leftMtrs.move_velocity(leftMtrStorage.output);
        rightMtrs.move_velocity(rightMtrStorage.output);

        if (abs(leftMtrStorage.error) < 0.5 && abs(rightMtrStorage.error) < 0.5)
            break;
        if (leftMtrStorage.timeSinceLastMovement >= 450 && rightMtrStorage.timeSinceLastMovement >= 450)
            break;

        pros::delay(15);
    }
    leftMtrs.move_velocity(0);
    rightMtrs.move_velocity(0);
}

/*
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
*/

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

    // cout << "desiredPoint1 " << tempStorage.desiredPoint1 << endl;

    // makes sure everything is in terms of 360 deg
    if (tempStorage.desiredPoint1 < 360)
        tempStorage.desiredPoint2 = tempStorage.desiredPoint1 + 360;
    else
        tempStorage.desiredPoint2 = tempStorage.desiredPoint1 - 360;

    // cout << "desiredPoint2 " << tempStorage.desiredPoint2 << endl;

    double desiredPoint;

    if (abs(currentHeading - tempStorage.desiredPoint1) < abs(currentHeading - tempStorage.desiredPoint2))
        desiredPoint = tempStorage.desiredPoint1;
    else
        desiredPoint = tempStorage.desiredPoint2;

    // cout << "current heading is " << currentHeading << " desiredPoint is " << desiredPoint << endl;

    // finding shortest direction
    // cout << "error 1: " << desiredHeading - currentHeading << " error 2: " << currentHeading - desiredHeading << endl;
    // if (abs(desiredHeading - currentHeading) < abs(currentHeading - desiredHeading))

    tempStorage.error = desiredPoint - currentHeading;

    cout << "Error: " << tempStorage.error << "Desired point used: " << desiredPoint << "Start input is: " << tempStorage.startInput << endl;

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

    // setting lastInput
    tempStorage.lastInput = currentHeading;

    return tempStorage;
}

PID_Holder_Turn findPIDOutputTurn(double turnAmount, double maxVelocity, PID_Holder_Turn input)
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

    // cout << "desiredPoint1 " << tempStorage.desiredPoint1 << endl;

    // makes sure everything is in terms of 360 deg
    if (tempStorage.desiredPoint1 < 360)
        tempStorage.desiredPoint2 = tempStorage.desiredPoint1 + 360;
    else
        tempStorage.desiredPoint2 = tempStorage.desiredPoint1 - 360;

    // cout << "desiredPoint2 " << tempStorage.desiredPoint2 << endl;

    double desiredPoint;

    if (abs(currentHeading - tempStorage.desiredPoint1) < abs(currentHeading - tempStorage.desiredPoint2))
        desiredPoint = tempStorage.desiredPoint1;
    else
        desiredPoint = tempStorage.desiredPoint2;

    // cout << "current heading is " << currentHeading << " desiredPoint is " << desiredPoint << endl;

    // finding shortest direction
    // cout << "error 1: " << desiredHeading - currentHeading << " error 2: " << currentHeading - desiredHeading << endl;
    // if (abs(desiredHeading - currentHeading) < abs(currentHeading - desiredHeading))

    tempStorage.error = desiredPoint - currentHeading;

    cout << "Error: " << tempStorage.error << endl;

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

    if (tempStorage.output > maxVelocity)
    {
        if (tempStorage.output > 0)
            tempStorage.output = maxVelocity;
        else
            tempStorage.output = -maxVelocity;
    }

    // setting lastInput
    tempStorage.lastInput = currentHeading;

    return tempStorage;
}

void turnDist(double desiredRotation)
{
    // look at get_rotation and get_heading
    // use set_heading and set_rotation in init

    PID_Holder_Turn inertialStorage;

    inertial.tare_heading();

    // cout << "Heading at auton start " << inertial.get_heading() << endl;

    while (true)
    {
        inertialStorage = findPIDOutputTurn(desiredRotation, inertialStorage);

        // cout << "error is " << inertialStorage.error << " output is " << inertialStorage.output << endl;
        // cout << "Desired point 1 " << inertialStorage.desiredPoint1 << " Desired point 2 " << inertialStorage.desiredPoint2 << endl;

        if (abs(inertialStorage.error) < 0.5)
            break;

        leftMtrs.move_velocity(inertialStorage.output);
        rightMtrs.move_velocity(-inertialStorage.output);

        pros::delay(15);
    }

    leftMtrs.move_velocity(0);
    rightMtrs.move_velocity(0);
}

void turnDist(double desiredRotation, double maxVelocity)
{
    // look at get_rotation and get_heading
    // use set_heading and set_rotation in init

    PID_Holder_Turn inertialStorage;

    inertial.tare_heading();

    // cout << "Heading at auton start " << inertial.get_heading() << endl;

    while (true)
    {
        inertialStorage = findPIDOutputTurn(desiredRotation, maxVelocity, inertialStorage);

        // cout << "error is " << inertialStorage.error << " output is " << inertialStorage.output << endl;
        // cout << "Desired point 1 " << inertialStorage.desiredPoint1 << " Desired point 2 " << inertialStorage.desiredPoint2 << endl;

        if (abs(inertialStorage.error) < 0.5)
            break;

        leftMtrs.move_velocity(inertialStorage.output);
        rightMtrs.move_velocity(-inertialStorage.output);

        pros::delay(15);
    }

    leftMtrs.move_velocity(0);
    rightMtrs.move_velocity(0);
}

/*
double *calcCurveDistance(double desiredDistance, double desiredDegree)
{
    const double rw = 103 / 16;

    double r = desiredDistance / desiredDegree - rw;

    double distanceArr[2];

    if (r > 0)
    {
        distanceArr[0] = r * desiredDegree;
        distanceArr[1] = (r + 2 * rw) * desiredDegree;
    }
    else
    {
        distanceArr[0] = (r + 2 * rw) * desiredDegree;
        distanceArr[1] = r * desiredDegree;
    }

    double a = distanceArr;

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
*/

void autonomous()
{
    // initTeamImg();

    leftMtrs.tare_position();
    rightMtrs.tare_position();
    inertial.tare_heading();

    // left awp
    if (closeSide)
    {
        moveDist(-25.0);
        intake.move_velocity(-200);
        pros::delay(500);
        wingRight.set_value(true);
        moveDist(25.0);
        intake.move_velocity(0);
        turnDist(-45.0);
        wingRight.set_value(false);
        turnDist(7.0);
        moveDist(22.0);
        turnDist(-7.0);
        moveDist(11.5);
        wingLeft.set_value(true);

        /*
        // r16
        wingLeft.set_value(true);
        moveDist(2.0, 100);
        turnDist(50.0);
        turnDist(-45.0);
        wingLeft.set_value(false);
        wingRight.set_value(false);
        moveDist(-5.5, 100);
        */

        /*
        wingRight.set_value(false);
        moveDist(25.5);
        turnDist(50.0);
        moveDist(25.0);
        wingLeft.set_value(true);
        wingRight.set_value(true);
        turnDist(-10.0);
        */
    }

    // right
    if (farSide)
    {
        moveDist(-37);
        turnDist(90.0);
        intake.move_velocity(-200);
        pros::delay(500);
        moveDist(1.0);
        moveDist(-3.0);
        pros::delay(500);
        moveDist(3.5);
        intake.move_velocity(0);

        turnDist(155.0);
        intake.move_velocity(200);
        moveDist(-21);
        pros::delay(500);
        moveDist(1.0);
        intake.move_velocity(0);

        turnDist(-55.0);
        moveDist(6.5);
        wingLeft.set_value(true);
        turnDist(45.0, 100);
        moveDist(12.0);
        wingLeft.set_value(false);

        moveDist(-2.0);
        turnDist(180.0);
        intake.move_velocity(-200);
        moveDist(4.0);

        // Getting in 2 triballs + alliance match load
        turnDist(180, 100);
        intake.move_velocity(0);
        wingLeft.set_value(true);
        wingRight.set_value(true);
        moveDist(20.0);

        /*
        Getting the 3rd triball + alliance match load
        turnDist(-180.0);
        intake.move_velocity(200);
        moveDist(-12.0);
        pros::delay(500);
        intake.move_velocity(0);

        turnDist(170);
        moveDist(-5.0);
        intake.move_velocity(-200);
        pros::delay(500);
        moveDist(5.0);
        intake.move_velocity(200);
        turnDist(180);
        wingLeft.set_value(true);
        wingRight.set_value(true);
        moveDist(24.0);
        */

        /*
        turnDist(140.0);
        moveDist(-6.0);
        intake.move_velocity(-200);
        pros::delay(500);
        turnDist(-75.0);
        intake.move_velocity(200);
        moveDist(-3.0);
        pros::delay(500);
        intake.move_velocity(0);
        moveDist(12.0);
        moveDist(-5.0);
        turnDist(180);
        intake.move_velocity(-200);
        moveDist(-5.0);

        intake.move_velocity(200);
        pros::delay(250);
        moveDist(-2.0, 50);
        pros::delay(500);
        intake.move_velocity(0);
        moveDist(34.0);
        turnDist(-35.0);
        moveDist(14.0, 100);
        wingLeft.set_value(true);
        wingRight.set_value(true);
        turnDist(-35.0, 50);
        wingLeft.set_value(false);
        wingRight.set_value(false);
        turnDist(180.0);
        moveDist(-24.0);
        */
    }

    // skills
    if (skills)
    {
        moveDist(20.0);
        turnDist(110.0);
        moveDist(-3.0);
        turnDist(5.0);

        int numShots = 0;
        while (numShots < 46)
        {
            moveCata2(catapult.get_position() + 180, 25);

            numShots++;
        }
        moveDist(80.0);
        moveDist(25);

        wingLeft.set_value(true);
        wingRight.set_value(true);
        turnDist(-45.0);
        moveDist(5.0);
        turnDist(45.0);
        moveDist(25);
    }

    if (disable)
        moveDist(12.0);
}