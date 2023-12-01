#include "main.h"
#include "lib/Robot.h"

void opcontrol() {
    Robot::threading.start("driver", Robot::driver_thread);
}
