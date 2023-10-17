#pragma once

#include "Odometry.h"
#include "../controllers/PID.h"
#include <array>
#include <cmath>
#include <deque>

class Drive {
    public:
        Drive();

        void brake(pros::motor_brake_mode_e_t);

        void move(double power, double turn);
        void move_to(Pose target, double moveAcc, double turnAcc, double maxspeed = 127, int timeout= 5000);
        void rotate_to(double targetHeading, double turnAcc, double maxSpeed = 127);
};
