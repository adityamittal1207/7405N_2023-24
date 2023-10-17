#include "main.h"
#include "lib/Robot.h"

void opcontrol() {
    pros::lcd::print(0, "CATA POSITION: %.2f", 1);
    // Robot::threading.start("driver", Robot::driver_thread);
    // printf("jawn\n");
    // while(true){
    //     printf("FW Instances %d\n", FlyWheel::instances);
    //     pros::delay(20);
    // }
}
