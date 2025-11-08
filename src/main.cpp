#include "main.h"
#include "robot-config.hpp"
#include "pros/motors.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

// LemLib includes
#include "lemlib/lemlib.hpp"  
#include "lemlib/asset.hpp"



// Load Paths 
ASSET(part1_txt);


void autonomous() {
    
    intakeMain.move(10000);
    chassis.setPose(0, 0, 0);
    chassis.follow(part1_txt, 1, 5);
    
    auto pose = chassis.getPose();
    chassis.moveToPose(pose.x - 20, pose.y, 90, 2000);
}