#pragma once

#include <atomic>
#include <cmath>

class Intake{
     private:
        
     public:

        std::atomic<int> pneu_state = 1; // 0 = down, 1 = up

        std::atomic<int> motor_state = 0; // 0 = idle, 1 = running

        std::atomic<bool> pneu_locked = false;

        std::atomic<bool> motor_locked = false;

        void pneulock_intake(){pneu_locked = true;};

        void pneuunlock_intake(){pneu_locked = false;};

        void motorlock_intake(){motor_locked = true;};

        void motorunlock_intake(){motor_locked = false;};

        void lift();

        void unlift();

        void intake();

        void outtake();

        void stopintake();

};