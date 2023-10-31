#include "api.h"

pros::Motor frontLeftMtr(1, pros::E_MOTOR_GEAR_GREEN, false,
                         pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor middleLeftMtr(2, pros::E_MOTOR_GEAR_GREEN, false,
                          pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor backLeftMtr(3, pros::E_MOTOR_GEAR_GREEN, true,
                        pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group leftMtrs({frontLeftMtr, middleLeftMtr, backLeftMtr});

pros::Motor frontRightMtr(8, pros::E_MOTOR_GEAR_GREEN, true,
                          pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor middleRightMtr(9, pros::E_MOTOR_GEAR_GREEN, true,
                           pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor backRightMtr(10, pros::E_MOTOR_GEAR_GREEN, false,
                         pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group rightMtrs({frontRightMtr, middleRightMtr, backRightMtr});

pros::Motor catapult(21, pros::E_MOTOR_GEAR_RED, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::ADIDigitalOut wingLeft('B');
pros::ADIDigitalOut wingRight('A');

pros::Motor intake(7, pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::IMU inertial(13);