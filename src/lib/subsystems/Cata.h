#pragma once

#include <atomic>
#include <cmath>

class Cata{
     private:
     public:

        std::atomic<int> state = 0; // 0 = bottom, 1 = shooting/reloading, 2 = mid

        std::atomic<bool> locked;
        std::atomic<double> final_angle = 0;

        int get_state(){return state;};

        void lock_cata();

        void unlock_cata(); 

// Shoot Functions

        void shoot();

        void spamfire();

        void shoot_to_mid();

        void shoot_to_end();

// Reload Functions

        void full_reload(){final_angle=93;state=0;};

        void reload_to_mid(){final_angle=70;state=2;};

        void reload_to_angle();

};