#include "Drive.h"
#include "../Robot.h"

Drive::Drive() {}


void Drive::brake(pros::motor_brake_mode_e_t brake) {
    Robot::FL.set_brake_mode(brake);
    Robot::FR.set_brake_mode(brake);
    Robot::BL.set_brake_mode(brake);
    Robot::BR.set_brake_mode(brake);
    Robot::CR.set_brake_mode(brake);
    Robot::CL.set_brake_mode(brake);
}

void Drive::move(double power, double turn) {
    Robot::FL = power + turn;
    Robot::BL = power + turn;
    Robot::CL = power + turn;
    Robot::FR = power - turn;
    Robot::CR = power - turn;
    Robot::BR = power - turn;
}

void Drive::move_to(Pose target, double moveAcc, double turnAcc, double maxspeed, int timeout) {
    Pose curPos = Robot::odometry.getPose();

//    double curPosHeading = std::fmod(curPos.theta, 180.0) - 180.0 * std::round(curPos.theta / (360.0));
    double curPosHeading = curPos.theta;

    double angleToPoint = util::to_deg(curPos.angleTo(target));
    double headingErr = angleToPoint - curPosHeading;
    double direction = std::cos(util::to_rad(headingErr)) > 0 ? 1 : -1;
    if (direction < 0) { headingErr = headingErr > 0 ? headingErr - 180.0 : headingErr + 180.0; }
    double moveErr = curPos.distanceTo(target);

    double moveCompleteBuff = 0;

    int i = 0;
    int time = 0;

    while (headingErr > turnAcc || moveCompleteBuff < 5) {
        i++;
        curPos = Robot::odometry.getPose();
        moveErr = curPos.distanceTo(target);

//        curPosHeading = std::fmod(curPos.theta, 180.0) - 180.0 * std::round(curPos.theta / (360.0));
        curPosHeading = curPos.theta;
        if (moveErr > 6.0) { angleToPoint = util::to_deg(curPos.angleTo(target)); }
        headingErr = angleToPoint - curPosHeading;
        double realHeadingErr = util::to_deg(curPos.angleTo(target)) - curPosHeading;
        direction = std::cos(util::to_rad(realHeadingErr)) > 0 ? 1 : -1;
        if (direction < 0) { headingErr = headingErr > 0 ? headingErr - 180.0 : headingErr + 180.0; }

        if (std::fabs(headingErr) > turnAcc) {
            double turnSpeed = Robot::turn.get_value(headingErr);
            // if (i % 3 == 0) {
            //     std::cout << "curPos: " << curPos.toString() << ", target: " << target.toString() << ", ";
            //     std::cout << "direction: " << direction << ", moveErr: " << moveErr << ", ";
            //     std::cout << "targetHeading: " << util::to_deg(curPos.angleTo(target)) << ", ";
            //     std::cout << "headingErr: " << headingErr << ", turnSpeed: " << turnSpeed << std::endl;
            // }
            // printf("turning turn move\n");
            move(0, turnSpeed);
        } else {
            if (moveErr > moveAcc) {
                moveCompleteBuff = 0;
            } else {
                moveCompleteBuff += 1;
            }
            double turnSpeed = Robot::turn.get_value(headingErr);
            double moveSpeed = Robot::power.get_value(moveErr);

            double speedSum = moveSpeed + std::fabs(turnSpeed);
            double maxMoveSpeed = moveSpeed * (127.0 / speedSum);
            double maxTurnSpeed = turnSpeed * (127.0 / speedSum);

            moveSpeed = moveSpeed < maxMoveSpeed ? moveSpeed : maxMoveSpeed;
            turnSpeed = std::fabs(turnSpeed) < std::fabs(maxTurnSpeed) ? turnSpeed : maxTurnSpeed;
            if (i % 5 == 0) {
                std::cout << "curPos: " << curPos.toString() << ", target: " << target.toString() << ", "
                          << "moveCompleteBuff: " << moveCompleteBuff << ", "
                          << "moveErr: " << moveErr << ", direction: " << direction << ", moveSpeed: " << moveSpeed << ", "
                          << "headingErr: " << headingErr << ", turnSpeed: " << turnSpeed
                          << std::endl;
            }

            if(std::abs(moveSpeed) > maxspeed) {moveSpeed = moveSpeed < 0 ? -maxspeed : maxspeed; }
            move(moveSpeed * direction, turnSpeed);
        }

        if(time > timeout) { //break out of that motherfucker
            break;
        }

        time += 5;
        pros::delay(5);
    }

    printf("done");
    Robot::power.reset();
    Robot::turn.reset();
    move(0, 0);

    brake(pros::E_MOTOR_BRAKE_COAST);

}

void Drive::rotate_to(double targetHeading, double turnAcc, double maxSpeed) {
    Pose curPos = Robot::odometry.getPose();
    double curPosHeading = std::fmod(curPos.theta, 180.0) - 180.0 * std::round(curPos.theta / (360.0));
    double headingErr = targetHeading - curPosHeading;
    if (std::fabs(headingErr) > 180.0) { headingErr = headingErr > 0.0 ? headingErr - 360.0 : headingErr + 360.0; }

    int i = 0;
    double turnCompleteBuff = 0;
    while (turnCompleteBuff < 5) {
        i++;
        if (std::fabs(headingErr) > turnAcc) {
            turnCompleteBuff = 0;
        } else {
            turnCompleteBuff += 1;
        }
        curPos = Robot::odometry.getPose();
        curPosHeading = std::fmod(curPos.theta, 180.0) - 180.0 * std::round(curPos.theta / (360.0));
        headingErr = targetHeading - curPosHeading;
        if (std::fabs(headingErr) > 180.0) { headingErr = headingErr > 0.0 ? headingErr - 360.0 : headingErr + 360.0; }

        double turnSpeed = Robot::turn.get_value(headingErr);

        if (i % 3 == 0) {
            std::cout << "curPos: " << curPos.toString() << ", targetHeading: " << targetHeading << ", "
                      << "turnCompleteBuff: " << turnCompleteBuff << ", "
                      << "headingErr: " << headingErr << ", turnSpeed: " << turnSpeed
                      << std::endl;
            printf("turning turn function\n");
        }

        if(std::abs(turnSpeed) > maxSpeed) {turnSpeed = turnSpeed < 0 ? -maxSpeed : maxSpeed; }

        move(0, turnSpeed);
    }

    printf("turn done");
    Robot::turn.reset();
    move(0, 0);
    brake(pros::E_MOTOR_BRAKE_COAST);

}


