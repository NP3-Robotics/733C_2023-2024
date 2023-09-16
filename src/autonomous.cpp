#include "main.h"
#include <iostream>

#include "my_stuff/port_declerations.h"

#define PI 3.141592653589793238462643383279502884L
using namespace std;

// kp e + ki integral(e) + kd de/dt
// Added PID code
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

PID_Holder leftMtrStorage;
PID_Holder rightMtrStorage;

void setInitMtrStuff()
{
    leftMtrStorage.left = true;
    leftMtrStorage.moving = false;

    rightMtrStorage.left = false;
    rightMtrStorage.moving = false;
}

double kp = 1;
double ki = 0.5;
double kd = 10;

double avgEncoder(std::vector<double> mtr)
{
    std::cout << mtr[0] << " " << mtr[1] << " " << mtr[2] << endl;
    double avgMtrEncoder =
        (mtr[0] + mtr[1] + mtr[2]) / 3;

    return avgMtrEncoder;
}

double calcDistanceTravelled(double encoderValue)
{
    // 2 pi r
    //  r = 2
    // single encoder value = 2 pi r deg / 360 = pi deg / 90

    double distTravelled = PI * encoderValue / 90;

    return distTravelled;
}

double findPIDOutput(int desiredDistance, PID_Holder &mtrStorage)
{
    double mtrPos;
    if (mtrStorage.left)
        mtrPos = avgEncoder(leftMtrs.get_positions());
    else
        mtrPos = avgEncoder(rightMtrs.get_positions());

    if (!(mtrStorage.moving))
    {
        mtrStorage.startInput = mtrPos;
        mtrStorage.moving = true;
    }

    double changeInEncoder = mtrPos - mtrStorage.startInput;

    double distTravelled = calcDistanceTravelled(changeInEncoder);

    mtrStorage.error = desiredDistance - distTravelled;

    mtrStorage.errorAcc += mtrStorage.error;

    double output = kp * mtrStorage.error + ki * mtrStorage.errorAcc + kd * (distTravelled - mtrStorage.lastInput) / 100;

    if (output > 200)
        output = 200;
    if (output < -200)
        output = -200;

    return output;
}

void moveDist(int desiredDistance)
{
    while (true)
    {
        leftMtrStorage.output = findPIDOutput(desiredDistance, leftMtrStorage);
        rightMtrStorage.output = findPIDOutput(desiredDistance, rightMtrStorage);

        /*
        if (abs(leftMtrStorage.error + rightMtrStorage.error) / 2 < 0.5)
        {

            std::cout << "loop ended" << endl;
            leftMtrs.move(0);
            rightMtrs.move(0);

            leftMtrStorage.output = 0;
            rightMtrStorage.output = 0;
            break;
        }
        */

        // std::cout << "loop still going" << endl;
        std::cout << "avg error is "
                  << abs(leftMtrStorage.error + rightMtrStorage.error) / 2 << endl;

        printf("Left output is %d. Right output is %d. \n", leftMtrStorage.output, rightMtrStorage.output);

        leftMtrs.move_velocity(leftMtrStorage.output);
        rightMtrs.move_velocity(rightMtrStorage.output);
        pros::delay(100);
    }

    leftMtrStorage.moving = false;
    rightMtrStorage.moving = false;
}

void autonomous()
{
    std::cout << "in auton" << endl;
    leftMtrs.tare_position();
    rightMtrs.tare_position();

    // leftMtrs.move_velocity(-200);
    // rightMtrs.move_velocity(-200);

    moveDist(10.0);

    pros::delay(1000);
}
