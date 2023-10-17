#pragma once

#include <atomic>
#include <cmath>

class Cata{
     private:
     public:

        std::atomic<int> state = 0; // 0 = bottom, 1 = shooting/reloading, 2 = mid

        std::atomic<bool> locked;

        int get_state(){return state;};

        void lock_cata();

        void unlock_cata(); 

// Shoot Functions

        void shoot();

        void shoot_to_mid();

        void shoot_to_end();

// Reload Functions

        void full_reload(){reload_to_angle(89.92);};

        void reload_to_mid(){reload_to_angle(78);};

        void reload_to_angle(double final_angle);

};