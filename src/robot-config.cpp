#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"


// left motor group
pros::MotorGroup leftMotor({-1, 2, -3}, pros::MotorGears::green);
// right motor group
pros::MotorGroup rightMotor({4, -5, 6}, pros::MotorGears::green);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotor, // left motor group
                              &rightMotor, // right motor group
                              13, // 13 inch track width
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
lemlib::TrackingWheel horizontal(&horizontalEncoder, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel
lemlib::TrackingWheel vertical(&verticalEncoder, lemlib::Omniwheel::NEW_275, -2.5); 

// odometry settings
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    &horizontal, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
    &imu // inertial sensor
);

// lateral PID controller
// PID for going forward/backward
lemlib::ControllerSettings lateralController(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
                                            );
                                            
// angular PID controller
// PID for turning
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                                0, // integral gain (kI)
                                                10, // derivative gain (kD)
                                                3, // anti windup
                                                1, // small error range, in degrees
                                                100, // small error range timeout, in milliseconds
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