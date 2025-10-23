#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"

// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Left and right motor groups
pros::MotorGroup leftMotors({-11, 13, -12});
pros::MotorGroup rightMotors({1, -3, 2});

// Other motors
pros::Motor outake(-19);
pros::Motor outake_control(-20);
pros::Motor intake(16);

// Pneumatics
pros::ADIPneumatics doinker('A', true);
pros::ADIPneumatics thingy('C', true); // new pneumatic on port B

// Drivetrain settings
lemlib::Drivetrain drivetrain(
    &leftMotors,                  // left motor group
    &rightMotors,                 // right motor group
    10.5,                         // track width in inches
    lemlib::Omniwheel::NEW_325,   // wheel type
    1000,                         // drivetrain RPM
    2                             // horizontal drift
);

// Lateral PID controller
lemlib::ControllerSettings lateralController(
    7, 0, 25, 0, 0.75, 120, 2.0, 450, 15
);

// Angular PID controller
lemlib::ControllerSettings angularController(
    2, 0, 18, 0, 1, 120, 3, 500, 0
);

// Input curves
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

// Odom sensors
lemlib::OdomSensors sensors(
    nullptr,   // vertical tracking wheel 1
    nullptr,   // vertical tracking wheel 2
    nullptr,   // horizontal tracking wheel
    nullptr,   // front tracking wheel (optional)
    nullptr    // inertial sensor
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
void opcontrol() {

    while (true) {
        // --- Tank drive at full speed ---
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        int leftRPM = leftY * 200 / 127;
        int rightRPM = rightY * 200 / 127;
        chassis.tank(leftRPM, rightRPM);

        // --- Intake ---
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
            intake.move_velocity(200);
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
            intake.move_velocity(-200);
        else
            intake.move_velocity(0);

        // --- Basket roller / outake ---
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
            outake.move_velocity(200);
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A))
            outake.move_velocity(-200);
        else
            outake.move_velocity(0);

        // --- High roller / outake_control ---
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
            outake_control.move_velocity(200);
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B))
            outake_control.move_velocity(-200);
        else
            outake_control.move_velocity(0);

        // --- Pneumatics ---
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X))
            doinker.set_value(true);
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
            doinker.set_value(false);

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
            thingy.set_value(true); // extend
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
            thingy.set_value(false); // retract

        pros::delay(25);
    }
}
