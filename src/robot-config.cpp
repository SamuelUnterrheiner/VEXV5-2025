#include "lemlib/lemlib.hpp"
#include "main.h"
#include "pros/motors.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

#include "lemlib/asset.hpp"
#include  "robot-config.hpp"

// Left and right motor groups
pros::MotorGroup leftMotors({11, -14, 13});
pros::MotorGroup rightMotors({-1, 6, -3});

// Drivetrain settings
lemlib::Drivetrain drivetrain(
    &leftMotors,                  // left motor group
    &rightMotors,                 // right motor group
    11.8,                         // track width in inches
    lemlib::Omniwheel::NEW_325,   // wheel type
    360,                          // drivetrain RPM
    2                             // horizontal drift (tweak if needed)
);
// Other motors
pros::Motor intakeMain(18);
pros::Motor basketRoller(20);
pros::Motor highRoller(19);

// Rotation Sensors
pros::Rotation vertical(15);
pros::Rotation horizontal(5);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical, lemlib::Omniwheel::NEW_275, 0, 0);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal, lemlib::Omniwheel::NEW_275, 0, 0);

//IMU
pros::Imu imu(1);
// Lateral PID controller
lemlib::ControllerSettings lateralController(
    7, 0, 25, 0, 0.75, 120, 2.0, 450, 15
);

// Angular PID controller
lemlib::ControllerSettings angularController(
    2, 0, 18, 0, 1, 120, 3, 500, 0
);

// Input curves (makes joystick smoother)
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

lemlib::OdomSensors sensors(
    &vertical_tracking_wheel,  // vertical
    nullptr,                   // second vertical
    &horizontal_tracking_wheel,// horizontal
    nullptr,                   // front
    nullptr                    // inertial
);
// Chassis setup
lemlib::Chassis chassis(
    drivetrain,
    lateralController,
    angularController,
    sensors,
    &throttleCurve,
    &steerCurve
);