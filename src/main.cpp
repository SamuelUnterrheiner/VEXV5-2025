#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({1, 12, 13});
pros::MotorGroup rightMotors({-15, -14, -3});

pros::Motor intake(2);

lemlib::Drivetrain drivetrain(
    &leftMotors,                  // left motor group
    &rightMotors,                 // right motor group
    10.5,                         // track width in inches
    lemlib::Omniwheel::NEW_325,   // wheel type
    1000,                         // drivetrain RPM
    2                             // horizontal drift
);

lemlib::ControllerSettings lateralController(
    7, 0, 25, 0, 0.75, 120, 2.0, 450, 15
);

lemlib::ControllerSettings angularController(
    2, 0, 18, 0, 1, 120, 3, 500, 0
);

lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

lemlib::OdomSensors sensors(
    nullptr,   // vertical tracking wheel 1
    nullptr,   // vertical tracking wheel 2
    nullptr,   // horizontal tracking wheel
    nullptr,   // front tracking wheel (optional)
    nullptr    // inertial sensor
);

lemlib::Chassis chassis(
    drivetrain,
    lateralController,
    angularController,
    sensors,
    &throttleCurve,
    &steerCurve
);
void opcontrol() {

    while (true) {
        int drive = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        int left = -drive + -turn;
        int right = -drive - -turn;

        int leftRPM = left * 200 / 127;
        int rightRPM = right * 200 / 127;
        chassis.tank(leftRPM, rightRPM);

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
            intake.move_velocity(200);
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
            intake.move_velocity(-200);
        else
            intake.move_velocity(0);

        pros::delay(25);
    }
}
