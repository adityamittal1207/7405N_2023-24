#include "main.h"
#include "lib/Robot.h"
#define IMUDELAYTIME 5000


void initialize() { 
    pros::lcd::initialize();

    pros::delay(100);

    Robot::IMU.reset();
    int time = 0;
    while(Robot::IMU.is_calibrating()) {
        time += 5;
        if(time > IMUDELAYTIME)
            break;
        pros::delay(5);
    }

    Robot::cata_pause=false;
    Robot::INT_EXP.set_value(false);
    Robot::threading.start("display", Robot::display_thread);
    Robot::threading.start("controller", Robot::controller_thread);
    Robot::threading.start("odometry", Robot::odom_thread);
    Robot::threading.start("driver", Robot::driver_thread);

}

void competition_initialize() {}

void disabled() {}