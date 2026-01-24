#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"

pros::Rotation horizontalEncoder(17);
pros::Rotation verticalEncoder(18);

lemlib::TrackingWheel horizontal(
    &horizontalEncoder,
    lemlib::Omniwheel::NEW_275,
    3.5
);

lemlib::TrackingWheel vertical(
    &verticalEncoder,
    lemlib::Omniwheel::NEW_275,
    4.0
);

pros::IMU imu(16);

lemlib::OdomSensors sensors(
    &vertical,
    nullptr,
    &horizontal,
    nullptr,
    &imu
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
    &leftMotors,
    &rightMotors,
    10.5,
    lemlib::Omniwheel::NEW_325,
    1000,
    2
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

bool lowerintakeforwards = false;
bool lowerintakereverse = false;
bool intakeforwards = false;
bool intakereverse = false;

bool lastL1 = false;
bool lastL2 = false;
bool lastR1 = false;
bool lastR2 = false;
bool lastUp = false;
bool lastDown = false;

void opcontrol() {
    while (true) {
        int drive = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        int left = -drive + -turn;
        int right = -drive - -turn;

        int leftRPM = left * 200 / 127;
        int rightRPM = right * 200 / 127;

        chassis.tank(leftRPM, rightRPM);

        bool L1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool L2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        bool R1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool R2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool UP = master.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
        bool DOWN = master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);

        if (L1 && !lastL1) {
            lowerintakeforwards = !lowerintakeforwards;
            lowerintakereverse = false;
        }

        if (L2 && !lastL2) {
            lowerintakereverse = !lowerintakereverse;
            lowerintakeforwards = false;
        }

        if (R1 && !lastR1) {
            intakeforwards = !intakeforwards;
            intakereverse = false;
        }

        if (R2 && !lastR2) {
            intakereverse = !intakereverse;
            intakeforwards = false;
        }

        if (lowerintakeforwards) {
            lowerintake.move_velocity(200);
        } else if (lowerintakereverse) {
            lowerintake.move_velocity(-200);
        } else {
            lowerintake.move_velocity(0);
        }

        if (intakeforwards) {
            higherintake.move_velocity(200);
        } else if (intakereverse) {
            higherintake.move_velocity(-200);
        } else {
            higherintake.move_velocity(0);
        }

        if (UP && !lastUp) {
            frontbarState = !frontbarState;
            frontbar.set_value(frontbarState);
        }

        if (DOWN && !lastDown) {
            backoutletState = !backoutletState;
            backoutlet.set_value(backoutletState);
        }

        lastL1 = L1;
        lastL2 = L2;
        lastR1 = R1;
        lastR2 = R2;
        lastUp = UP;
        lastDown = DOWN;

        pros::delay(25);
    }
}
