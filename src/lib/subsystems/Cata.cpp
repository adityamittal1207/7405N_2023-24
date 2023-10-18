#include "Cata.h"
#include "../Robot.h"
#include "pros/rtos.hpp"

void Cata::lock_cata() {
    locked = true;
}

void Cata::unlock_cata() {
    locked = false;
}

void Cata::shoot() {
    if(!locked) {
        state = 1;
        Robot::CATA.move(127);
        pros::delay(1000);
        Robot::CATA.move(0);
    }   
}

void Cata::shoot_to_mid(){
    shoot();
    reload_to_mid();
    state = 2;
}

void Cata::shoot_to_end(){
    shoot();
    full_reload();
    state = 0;
}


void Cata::reload_to_angle() {
    if(!locked) {
        state = 1;
        double angle = (double) (Robot::CATAROT.get_position())/100.0;
        double error = final_angle - angle;
        double power = Robot::cata_power.get_value(error);

        power = std::min(127.0, std::max(0.0, power));

        printf("%.2f: \t angle: %.2f, error: %.2f, power: %.2f\n", (double) pros::millis(), (double) final_angle, error, power);

        Robot::CATA.move(power);
    }
}
