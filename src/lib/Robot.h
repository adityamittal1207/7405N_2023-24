#pragma once

#include "main.h"

// Drive Imports
#include "drive/Drive.h"

// Subsystem Imports
#include "pros/rotation.hpp"
#include "subsystems/Cata.h"
#include "subsystems/Intake.h"
// Util Imports
#include "util/Threading.h"
#include "util/Util.h"
#include "util/selectionScreen.h"
#include "util/Display.h"
#include "util/selectionScreen.h"

// Other
#include <map>
#include <functional>
#include <string>
#include "lemlib/api.hpp"

class Robot {
    public:
        static std::atomic<bool> auton_done;
        static pros::Controller master;

        // Motors
        static pros::Motor FL;
        static pros::Motor CL;
        static pros::Motor BL;

        static pros::Motor FR;
        static pros::Motor CR;
        static pros::Motor BR;

        // Intake
        static pros::Motor INT;

        // CATA
        static pros::Motor CATA;

        // Cata rotation sensor

        static pros::Rotation CATAROT;

        // Sensors
        static pros::Imu IMU;

        // Intake Pistons
        // static pros::ADIDigitalOut LEFTWING;
        // static pros::ADIDigitalOut RIGHTWING;
        static pros::ADIDigitalOut WINGS;
        static pros::ADIDigitalOut BLOCKER;

    // Drive
        static Drive drive;
        static Odometry odometry;
        static PID power;
        static PID turn;

        // Subsystems

        static Cata catapult;
        static Intake intake;
        static PID cata_power;
        static std::atomic<bool> cata_pause;


    // Utility
        static Threading threading;
        static TeamSelection teamSelection;


        static void driver_thread(void *ptr);
        static void display_thread(void *ptr);
        static void controller_thread(void *ptr);
        static void odom_thread(void *ptr);

    // Lemlib
        static lemlib::Drivetrain_t drivetrain;
        static lemlib::OdomSensors_t sensors;
        static lemlib::ChassisController_t lateralController;
        static lemlib::ChassisController_t angularController;
        static lemlib::ChassisController_t chassis;
};

