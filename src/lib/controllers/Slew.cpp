//
// Created by Zayn Rekhi on 10/10/22.
//

#include "Slew.h"

Slew::Slew(double accel_limit, double decel_limit) {
    accel_limit_ = accel_limit;
    decel_limit_ = decel_limit;
}

double Slew::limit(double val) {
    if (val - prev_val > accel_limit_) {}
}