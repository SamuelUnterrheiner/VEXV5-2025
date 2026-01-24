#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"


// Rotation sensors
pros::Rotation horizontalEncoder(17);
pros::Rotation verticalEncoder(18);

// Tracking wheels
lemlib::TrackingWheel horizontal(
    &horizontalEncoder,
    lemlib::Omniwheel::NEW_275, // CHANGE if 3.25"
    3.5 // inches from robot center (right = +, left = -)
);

lemlib::TrackingWheel vertical(
    &verticalEncoder,
    lemlib::Omniwheel::NEW_275, // CHANGE if 3.25"
    4.0 // inches from robot center (forward = +, backward = -)
);

pros::IMU imu(16);
// Odom sensors
lemlib::OdomSensors sensors(
    &vertical,   // vertical tracking wheel
    nullptr,     // second vertical (unused)
    &horizontal, // horizontal tracking wheel
    nullptr,     // front tracking wheel
    &imu         // IMU
);

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftMotors({12, 14, 3});
pros::MotorGroup rightMotors({-1, -15, -13});
pros::Motor lowerintake(10);
pros::Motor higherintake(2);
pros::adi::DigitalOut frontbar('A');
pros::adi::DigitalOut backoutlet('H');
bool frontbarState = false;
bool backoutletState = false;



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
            higherintake.move_velocity(200);
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
            higherintake.move_velocity(-200);
        else
            higherintake.move_velocity(0);

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
            lowerintake.move_velocity(200);
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
            lowerintake.move_velocity(-200);
        else
            lowerintake.move_velocity(0);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            frontbarState = !frontbarState;
            frontbar.set_value(frontbarState);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            backoutletState = !backoutletState;
            backoutlet.set_value(backoutletState);
        }
        pros::delay(25);
    }
}
