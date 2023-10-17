#include "Odometry.h"
#include "../Robot.h"

Odometry::Odometry(double horizontal_offset, double wheel_diameter_) : horizontal_offset_(horizontal_offset),
                                                                       wheel_circumference_(wheel_diameter_ * M_PI),
                                                                       cur_point(Pose()) {
    Robot::CL.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
    Robot::CR.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
    Robot::FL.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
    Robot::FR.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);

    Robot::CL.tare_position();
    Robot::CR.tare_position();
    Robot::FL.tare_position();
    Robot::FR.tare_position();
}



/**
 * Steps to updating Odometry:
 *   1. Get current encoder values
 *   2. Convert current encoder values to inches
 *   3. Update previous encoder values
 *   4. Calculate the difference in angle and the current angle
 *   5. Calculate the change in position (X and Y Coordinates)
 *   6. Rotate change in position by the change in angle
 *   7. Update X and Y coordinates
 */
void Odometry::update() {
    double curHeading = util::to_rad(Robot::IMU.get_rotation());
    double left = (Robot::CL.get_position() + Robot::FL.get_position()) / 2;
    double right = (Robot::CR.get_position() + Robot::FR.get_position()) / 2;

    double curLE = ((((((left / 50) * 360) / 6) * 3) / 4) / 360) * wheel_circumference_;
    double curRE = ((((((right / 50) * 360) / 6) * 3) / 4) / 360) * wheel_circumference_;

    double dLE = curLE - prev_encs[0];
    double dRE = curRE - prev_encs[1];

    prev_encs[0] = curLE;
    prev_encs[1] = curRE;

    double vertical_motion = (dRE + dLE) / 2;
    double posX = vertical_motion * std::sin(curHeading);
    double posY = vertical_motion * std::cos(curHeading);

    /* Updates current X and Y coordinates */
    cur_point.x += posX;
    cur_point.y += posY;
    cur_point.theta = util::to_deg(curHeading);
}
