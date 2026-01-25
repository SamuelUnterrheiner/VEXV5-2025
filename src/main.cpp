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
pros::Motor higherintake(-2);

pros::adi::DigitalOut frontbar('A');
pros::adi::DigitalOut backoutlet('H');

lemlib::Drivetrain drivetrain(
    &leftMotors,
    &rightMotors,
    10.5,
    lemlib::Omniwheel::NEW_325,
    1000,
    2
);

lemlib::ControllerSettings lateralController(
    7, 0, 5, 0, 0.75, 120, 2.0, 450, 15
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
void initialize() {}
void disabled() {}
void autonomous() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    imu.reset();
    while (imu.is_calibrating()) {
        pros::delay(10);
    }
    chassis.calibrate();
    chassis.setPose(0, 0, 0);
    master.clear();
	pros::Task screenTask{[&]() {
        while (true) {
			master.print(0,0,"X: %.3f", chassis.getPose().x);
			pros::delay(50);
			master.print(1,0,"Y: %.3f", chassis.getPose().y);
			pros::delay(50);
			master.print(2,0,"Theta: %.3f", chassis.getPose().theta);
			pros::delay(50);
        }
    }};
    // chassis.setPose(0, 0, 0);
    // chassis.moveToPose(12, 0, 0, 3000);
    leftMotors.move_voltage(6000);
    rightMotors.move_voltage(6000);
    pros::delay(1000);
    rightMotors.brake();
    leftMotors.brake();
}


void opcontrol() {
    autonomous();
    while (true) {
        pros::delay(25);
    }
}
