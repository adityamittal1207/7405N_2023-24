//
// Created by zayn on 7/12/21.
//

#include "main.h"
#include "PID.h"
#include <cmath>

/**
 * @brief Initialization of the PID class
 * @param p Sets the P coefficient kp
 * @param i Sets the I coefficient ki
 * @param d Sets the D coefficient kd
 * @param i_bound Sets the boundary for I
 */
PID::PID(double p, double i, double d, double i_bound, double minspeed_) {
    kp = p;
    ki = i;
    kd = d;
    integral_bound = i_bound;
    minspeed = minspeed_;


    error_sum = 0;      //Needed for Integral
    prev_error = 0;
}

/**
 * @brief Calculates the current speeds based on the error term
 * @param error - The distance from the current position and the target position
 * @return Sum of P, I, and D calculations: SPEED
 */
double PID::get_value(double error) {
    error_sum += error;

    double delta_error = error - prev_error;
    double derivative_of_error = delta_error / 5;

    double p_calc = kp * error;
    double i_calc = ki * error_sum;
    double d_calc = kd * derivative_of_error;

    if(std::abs(error) > integral_bound && integral_bound != 0) {
      i_calc = 0;
      error_sum = 0;
    }

    prev_error = error;
    double speed = p_calc + i_calc + d_calc;
    if(std::abs(speed) < minspeed && speed != 0) { speed = speed < 0 ? -speed : speed; }

    return speed;
}

void PID::set_value(double p, double i, double d, double i_bound, double minspeed_) {
    kp = p;
    ki = i;
    kd = d;
    integral_bound = i_bound;
    minspeed = minspeed_;
}

/**
 * @brief resets running pid values for next run
 */
void PID::reset() {
    error_sum = 0;
    prev_error = 0;
}
