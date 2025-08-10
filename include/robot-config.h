#pragma once
#include "lemlib/api.hpp"
#include "main.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

extern pros::Controller controller;
// motor groups
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::IMU imu;
extern pros::Rotation horizontalEncoder;
extern pros::Rotation verticalEncoder;

// lemlib defs
extern lemlib::TrackingWheel horizontal;
extern lemlib::TrackingWheel vertical;
extern lemlib::Drivetrain drivetrain;
extern lemlib::ControllerSettings lateralController;
extern lemlib::ControllerSettings angularController;
extern lemlib::OdomSensors sensors;
extern lemlib::ExpoDriveCurve throttleCurve;
extern lemlib::ExpoDriveCurve steerCurve;
extern lemlib::Chassis chassis;