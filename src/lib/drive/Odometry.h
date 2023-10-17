#pragma once

#include "main.h"
#include "../util/Pose.h"
#include "../util/Util.h"

#include <vector>
#include <string>

class Odometry {
    private:
        Pose cur_point;     

        double horizontal_offset_;
        double wheel_circumference_;

        std::array<double, 2> prev_encs;

    public:
        Odometry(double horizontal_offset, double wheel_diameter);
        void update();

        Pose getPose() { return cur_point; }
        void setPose(Pose point) { cur_point = point; }
};
