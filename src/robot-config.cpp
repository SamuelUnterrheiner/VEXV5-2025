#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"


// left motor group
pros::MotorGroup leftMotors({-1, 2, -3}, pros::MotorGears::green);
// right motor group
pros::MotorGroup rightMotors({4, -5, 6}, pros::MotorGears::green);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10.5, // 10.5 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              200, // rpm (green)
                              2 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(10);

// Tracking wheels
pros::Rotation horizontalEncoder(20); // Positive means that the encoder is forwards
pros::Rotation verticalEncoder(-19); // Negative means that the encoder is backwards

// Distance is the distance from the center of the robot to the tracking wheel
// horizontal tracking wheel
lemlib::TrackingWheel horizontal(&horizontalEncoder, lemlib::Omniwheel::NEW_275, -6.5);
// vertical tracking wheel
lemlib::TrackingWheel vertical(&verticalEncoder, lemlib::Omniwheel::NEW_275, -6.5); 

// odometry settings
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    &horizontal, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
    &imu // inertial sensor
);

// lateral PID controller
// PID for going forward/backward
lemlib::ControllerSettings lateralController(7, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              25, // derivative gain (kD)
                                              0, // anti windup
                                              0.75, // small error range, in inches
                                              120, // small error range timeout, in milliseconds
                                              2.0, // large error range, in inches
                                              450, // large error range timeout, in milliseconds
                                              15 // maximum acceleration (slew)
                                            );
                                            
// angular PID controller
// PID for turning
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                                0, // integral gain (kI)
                                                18, // derivative gain (kD)
                                                0, // anti windup
                                                1, // small error range, in degrees
                                                120, // small error range timeout, in milliseconds
                                                3, // large error range, in degrees
                                                500, // large error range timeout, in milliseconds
                                                0 // maximum acceleration (slew)
                                            );


// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);
// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateralController,
                        angularController,
                        sensors,
                        &throttleCurve, 
                        &steerCurve
);
// initialize function. Runs on program startup