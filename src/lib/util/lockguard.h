#pragma once
#include "api.h"

/**
 * @brief Simple lockguard for pros::Mutex.
 */
class LockGuard {
private:
    pros::Mutex* lock;

public:
    LockGuard(pros::Mutex& mutex) : lock(&mutex) { lock->take(TIMEOUT_MAX); }
    ~LockGuard() { lock->give(); }
};