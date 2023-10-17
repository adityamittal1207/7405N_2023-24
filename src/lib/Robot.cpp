#include "Robot.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "subsystems/Cata.h"

// ---------------- lil qiao ---------------- //


/* ========================================================================== */
/*                             Robot 🧠🤔                                     */
/* ========================================================================== */
// Controller
pros::Controller Robot::master(pros::E_CONTROLLER_MASTER);

// Motors
pros::Motor Robot::FL(3, true);  // Front Left: 3
pros::Motor Robot::CL(11, true); // Center Left: Port 5
pros::Motor Robot::BL(8, true); //  // Back Left: 8
pros::Motor Robot::FR(14, false);   // Front Right: Port 14
pros::Motor Robot::CR(10, false); // Center Right: Port 10
pros::Motor Robot::BR(5, false); // // Back Right: Port 5

// Intake
pros::Motor Robot::INT(6, true); //

// Cata
pros::Motor Robot::CATA(19);

// Cata rotation sensor
pros::Rotation Robot::CATAROT(20);

// Sensors
pros::IMU Robot::IMU(0);

// Expansion Pistons
pros::ADIDigitalOut Robot::INT1({{0, 'A'}});
pros::ADIDigitalOut Robot::INT2({{0, 'C'}});



/* ========================================================================== */
/*                               Drive 🚗 🏎️ 🚘                               */
/* ========================================================================== */
Drive Robot::drive;
Odometry Robot::odometry(5.5, 2.75);

PID Robot::power(6.9, 0.00001, 0, 5, 0);
PID Robot::turn(0, 0, 0, 0, 0);

/* ========================================================================== */
/*                              Subsystems 🦾🦿                               */
/* ========================================================================== */

Cata Robot::catapult;
Intake Robot::intake;

/* ========================================================================== */
/*                               Utility 🔨⛏ 🛠                               */
/* ========================================================================== */
Threading Robot::threading(100);
TeamSelection Robot::teamSelection = TeamSelection::UNKNOWN;

/* ========================================================================== */
/*                               Threads 🧵🪡                                 */
/* ========================================================================== */
void Robot::driver_thread(void *ptr) {
    while(true){
            // FLY.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

            int flyspeed_mode = 0;
            bool flyspeed_change = false;
            double flyspeed = 0;

            bool activate_triple_shot = false;
            int triple_shot_time = 0;
            int numShots = 0;
            int shot_time_start = 0;

            bool activate_single_shot = false;
            int single_shot_time = 0;

            bool activate_angle_change = false;

            INT.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

            // FL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                // CL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                // BL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                // FR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                // CR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                // BR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); 

            while(true) {
                //Drive
                
                int power = master.get_analog(ANALOG_LEFT_Y);
                int turn = master.get_analog(ANALOG_RIGHT_X);

//        if(std::abs(power) < 20) power = 0;
//        if(std::abs(turn) < 20) turn = 0;

                drive.move(power, util::dampen(turn));

                //Intake
                bool intakebutton = master.get_digital(DIGITAL_R2);
                bool outtakebutton = master.get_digital(DIGITAL_R1);

                if (intakebutton){
                    printf("intake\n");
                    intake.intake();
                }
                else if (outtakebutton){
                    intake.outtake();
                }
                else{
                    intake.stopintake();
                }

                bool cata = master.get_digital(DIGITAL_A);
                bool cata2 = master.get_digital(DIGITAL_B);
                if (cata2){
                    catapult.full_reload();
                }
                if (cata){
                    catapult.shoot();
                }
                pros::delay(5);
            }}
        pros::delay(5);
    }


void Robot::display_thread(void *ptr) {
    while (true) {
        Pose cur = Robot::odometry.getPose();

        double RE_val = CL.get_position();
        double BE_val = CR.get_position();

        double l_temp = (FL.get_temperature() + CL.get_temperature() + BL.get_temperature()) / 3;
        double r_temp = (FR.get_temperature() + CR.get_temperature() + BR.get_temperature()) / 3;

        pros::lcd::print(1, "Left: %.2f %.2f %.2f", FL.get_actual_velocity(), CL.get_actual_velocity(), BL.get_actual_velocity());
        pros::lcd::print(2, "Right: %.2f %.2f %.2f", FR.get_actual_velocity(), CR.get_actual_velocity(), BR.get_actual_velocity());
        pros::lcd::print(3, "Rotation Sensor Angle: %.2f", Robot::CATAROT.get_position());
        // pros::lcd::print(3, "FT: %.1f LT: %.1f RT: %.1f IT: %.1f ", FLY.get_temperature(), l_temp, r_temp, INT.get_temperature());
        pros::lcd::print(4, "X=%.2f, Y=%.2f, A=%.2f", cur.x, cur.y, cur.theta);
        pros::lcd::print(5, "%.2f %.2f %.2f %.2f", FL.get_position(), FR.get_position(), CL.get_position(), CR.get_position());
        pros::lcd::print(6, "Cata Temp %.2f", CATA.get_temperature());

        pros::delay(5);
    }
}

void Robot::controller_thread(void *ptr) {
    while (true) {
        pros::delay(50);
    }
}

void Robot::odom_thread(void *ptr) {
    CL.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
    CR.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
    FL.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
    FR.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);

    CL.tare_position();
    CR.tare_position();
    FL.tare_position();
    FR.tare_position();

    while(true) {
        odometry.update();
        pros::delay(5);
    }
}
