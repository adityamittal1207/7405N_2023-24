#pragma once

#include "../controllers/TBH.h"
#include <atomic>
#include <cmath>
#include <deque>

class FlyWheel{
     private:
        std::atomic<int> settle_count = 0;
        std::atomic<int> mode = 0;

        std::deque<double> bufferVel;
        int bufferVelSize = 10;

     public:
         std::atomic<double> target_speed;

        FlyWheel();

        double get_velocity();
        void set_velocity(double speed);
        bool reached_target(double thresh);

        void set_mode(int mode_) { mode = mode_; };
        int get_mode() { return mode; };

        void update();

};