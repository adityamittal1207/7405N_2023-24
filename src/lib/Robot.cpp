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
pros::Motor Robot::FL(2, true);  // Front Left: 3
pros::Motor Robot::CL(4, true); // Center Left: Port 16
pros::Motor Robot::BL(10, true); //  // Back Left: 8
pros::Motor Robot::FR(1, false);   // Front Right: Port 14 2.1
pros::Motor Robot::CR(3, false); // Center Right: Port 10 0.2
pros::Motor Robot::BR(7, false); // // Back Right: Port 5

// Intake
pros::Motor Robot::INT(19, true); //

// Cata
pros::Motor Robot::CATA(6);

// Cata rotation sensor
pros::Rotation Robot::CATAROT(5);

// Sensors
pros::IMU Robot::IMU(8);

// Expansion Pistons
// pros::ADIDigitalOut LEFTWING();
// pros::ADIDigitalOut RIGHTWING();
pros::ADIDigitalOut Robot::WINGS('A');

int old_angle = Robot::CATAROT.get_position();

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
std::atomic<bool> Robot::cata_pause;
PID Robot::cata_power(20, 0, 0, 0, 0);
int final_angle = 4543;

/* ========================================================================== */
/*                               Utility 🔨⛏ 🛠                               */
/* ========================================================================== */
Threading Robot::threading(100);
TeamSelection Robot::teamSelection = TeamSelection::UNKNOWN;

int counter = 0;

/* ========================================================================== */
/*                               Threads 🧵🪡                                 */
/* ========================================================================== */
void Robot::driver_thread(void *ptr) {
    Robot::threading.start("display", Robot::display_thread);
    Robot::threading.start("odometry", Robot::odom_thread);
    while(true){


            INT.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);


            while(true) {
                //Drive
                
                int power = master.get_analog(ANALOG_LEFT_Y);
                int turn = master.get_analog(ANALOG_RIGHT_X);

//        if(std::abs(power) < 20) power = 0;
//        if(std::abs(turn) < 20) turn = 0

                drive.move(power, util::dampen(turn));

                //Intake
                bool outtakebutton = master.get_digital(DIGITAL_R2);
                bool intakebutton = master.get_digital(DIGITAL_R1);


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

                // bool cata_reload_mid = master.get_digital(DIGITAL_A);
                // bool cata_reload_full = master.get_digital(DIGITAL_B);
                // bool cata_shoot = master.get_digital(DIGITAL_X);
                bool cata = master.get_digital(DIGITAL_L1);
                if(cata) {
                    if (CATAROT.get_position() - old_angle > 250){
                        Robot::CATA.move(0);
                        pros::delay(260);
                        Robot::CATA.move(127);
                    }
                    else{
                        if (final_angle - CATAROT.get_position() > -300){
                            Robot::CATA.move(105);
                        }
                        else{
                            Robot::CATA.move(127);
                        }
                    }
                    old_angle = CATAROT.get_position(); 
                }
                else{
                    Robot::CATA.move(0);
                }
                // if (intake.pneu_state == 1) {
                //     if (cata_reload_mid){
                //         catapult.reload_to_mid();
                //     }
                //     if (cata_reload_full){
                //         catapult.full_reload();
                //     }
                //     if(cata_shoot) {
                //         cata_pause = true;
                //         pros::delay(150);
                //         catapult.shoot();    
                //         cata_pause = false;
                //     }
                //     if (spamfire){
                //         cata_pause=true;
                //         CATA.move(80);
                //     }
                // }


                bool retract = master.get_digital_new_press(DIGITAL_DOWN);
                bool expand = master.get_digital_new_press(DIGITAL_UP);
                // bool leftwing = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT);
                // bool rightwing = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT);
                bool wingpneu = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2);
                if (wingpneu){
                    if (counter % 2 == 0) {
                        Robot::WINGS.set_value(true);
                    } else {
                        Robot::WINGS.set_value(false);
                    }
                    counter++;
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
    // while (true) {
    //     if (!Robot::cata_pause) Robot::catapult.reload_to_angle();
    //     pros::delay(50);
    // }
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
