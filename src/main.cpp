#include "main.h"
#include "lemlib/api.hpp"

pros::Motor left_front_motor(15, pros::E_MOTOR_GEARSET_06, true);
pros::Motor left_center_motor(14, pros::E_MOTOR_GEARSET_06, true);
pros::Motor left_back_motor(10, pros::E_MOTOR_GEARSET_06, true);
pros::Motor right_front_motor(7, pros::E_MOTOR_GEARSET_06, false);
pros::Motor right_center_motor(6, pros::E_MOTOR_GEARSET_06, false);
pros::Motor right_back_motor(9, pros::E_MOTOR_GEARSET_06, false);

pros::Motor intake(20); //
pros::Motor catapult(5);

pros::Imu inertial_sensor(11); // port 8


// Cata rotation sensor
pros::Rotation catapult_rotation(5);
pros::ADIDigitalOut wings('G'); //backwings
pros::ADIDigitalOut frontwings('H'); //frontwings

pros::MotorGroup left_side_motors({left_front_motor, left_center_motor, left_back_motor});
pros::MotorGroup right_side_motors({right_front_motor, right_center_motor, right_back_motor});

pros::Controller master(pros::E_CONTROLLER_MASTER);

lemlib::Drivetrain_t drivetrain {
        &left_side_motors, // left drivetrain motors
        &right_side_motors, // right drivetrain motors
        10, // track width
        3.25, // wheel diameter
        360 // wheel rpm
};

lemlib::TrackingWheel left_tracking_wheel(&left_side_motors, 2.75, -4.6, 343); // 2.75" wheel diameter, -4.6" offset from tracking center
lemlib::TrackingWheel right_tracking_wheel(&right_side_motors, 2.75, 4.6, 343); // 2.75" wheel diameter, 1.7" offset from tracking center



// odometry struct
lemlib::OdomSensors_t sensors {
        &left_tracking_wheel, // vertical tracking wheel 1
        &right_tracking_wheel, // vertical tracking wheel 2
        nullptr, // horizontal tracking wheel 1
        nullptr, // we don't have a second tracking wheel, so we set it to nullptr
        &inertial_sensor // inertial sensor
};

// forward/backward PID
lemlib::ChassisController_t lateralController {
        8, // kP
        30, // kD
        1, // smallErrorRange
        100, // smallErrorTimeout
        3, // largeErrorRange
        500, // largeErrorTimeout
        5 // slew rate
};

// turning PID
lemlib::ChassisController_t angularController {
        5, // kP
        40, // kD
        1, // smallErrorRange
        100, // smallErrorTimeout
        3, // largeErrorRange
        500, // largeErrorTimeout
        40 // slew rate
};

lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);


void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void move(double power, double turn) {
    left_front_motor = power + turn;
    left_center_motor = power + turn;
    left_back_motor = power + turn;
    right_front_motor = power - turn;
    right_center_motor = power - turn;
    right_back_motor = power - turn;
}

void move_drive(double power, double turn) {
    int left = power + turn;
    int right = power - turn;

    if (left < 0) left = left - 7;
    else if (left > 0) left = left + 7;

    if (right < 0) right = right - 7;
    else if (right > 0) right = right + 7;


    left_front_motor = left;
    left_center_motor = left;
    left_back_motor = left;
    right_front_motor = right;
    right_center_motor = right;
    right_back_motor = right;
}


void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f | y: %f", pose.x, pose.y, pose.theta); // print the x position
        pros::lcd::print(1, "H: %f", pose.theta); // print the x position


        pros::delay(10);
    }
}
static int dampen(int input) {
        double s = 40;
        double a = .60;
        double v = (127*a-127)/(-s*s+254*s-16129);
        double c = a - 2*v*s;
        double output;
        if (abs(input) < abs(s)) {
            output = a * input;
        }
        else {
            double x = abs(input);
            double y = -(s - x) * (c + v * (s + x)) + a * s;
            output = y * input / abs(input);
        }

    return (int)std::round(output);
}
void rotate_to(double targetHeading, double turnAcc, double maxSpeed) {
    double theta = inertial_sensor.get_rotation();
    double curPosHeading = std::fmod(theta, 180.0) - 180.0 * std::round(theta / (360.0));
    double headingErr = targetHeading - curPosHeading;
    double errorsum = 0;
    double turnSpeed = 0;
    if (std::fabs(headingErr) > 180.0) { headingErr = headingErr > 0.0 ? headingErr - 360.0 : headingErr + 360.0; }

    int i = 0;
    double turnCompleteBuff = 0;
    while (turnCompleteBuff < 30) {
        i++;
        if (std::fabs(headingErr) > turnAcc) {
            turnCompleteBuff = 0;
        } else {
            turnCompleteBuff += 1;
        }
        double theta = inertial_sensor.get_rotation();
        double curPosHeading = std::fmod(theta, 180.0) - 180.0 * std::round(theta / (360.0));
        headingErr = targetHeading - curPosHeading;

        if(headingErr < 10) errorsum += headingErr;

        if (std::fabs(headingErr) > 180.0) { headingErr = headingErr > 0.0 ? headingErr - 360.0 : headingErr + 360.0; }

        turnSpeed = headingErr * 1.2 + errorsum * 0.0001;

        // if (i % 3 == 0) {
        //     std::cout << "curPos: " << curPos.toString() << ", targetHeading: " << targetHeading << ", "
        //               << "turnCompleteBuff: " << turnCompleteBuff << ", "
        //               << "headingErr: " << headingErr << ", turnSpeed: " << turnSpeed
        //               << std::endl;
        //     printf("turning turn function\n");
        // }

        if(std::abs(turnSpeed) > maxSpeed) {turnSpeed = turnSpeed < 0 ? -maxSpeed : maxSpeed; }

        move(0, turnSpeed);
    }

    printf("turn done");
    move(0, 0);
}


void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate the chassis
    chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0
    pros::Task screenTask(screen); // create a task to print the position to the screen
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void skills() {
    // 1.75 -14.7


    chassis.moveTo(0, -14, 2000, 127); // move to the point (53, 53) with a timeout of 1000 ms
    

    pros::delay(250);

    chassis.turnTo(1.75, -14.7, 2000, 70);
    chassis.moveTo(1.75, -14.7, 2000, 70);

    pros::delay(25100000);


    chassis.moveTo(-4.5,-22, 750, 127);
    pros::delay(200);
    chassis.moveTo(-10, -32, 750, 127);
    pros::delay(200);
    chassis.moveTo(-4.5,-22, 750, 127);
    pros::delay(200);
    chassis.moveTo(-10, -32, 750, 127);
    pros::delay(200);

    chassis.moveTo(-3,-17, 1000, 70);

    chassis.turnTo(-10,-14, 1500, false, 80);

    chassis.moveTo(0, -20, 1000, 100);

    wings.set_value(true);

    catapult.move(127);

    pros::delay(30000);

    catapult.move(0);

    wings.set_value(false);

    pros::delay(500);

    intake.move(-127);

    chassis.moveTo(-17.0, -9.1, 2000, 50);

    chassis.turnTo(-42.14, -29.5, 1000, false, 80);

    chassis.moveTo(-42.14, -29.5, 2000, 80);

    chassis.turnTo(-55, -18.9, 1000, false, 70);

    intake.move(0);

    move(127, 0);
    pros::delay(800);
    frontwings.set_value(true);
    pros::delay(500);
    move(0, 0);

    move(127, 20);
    pros::delay(1000);
    move(-50,0);
    pros::delay(1000);
    move(127,15);
    pros::delay(1000);
    move(-75, 0);
    pros::delay(800);
    move(127,30);
    pros::delay(1000);
    move(-100,0);
    pros::delay(750);
    move(127,35);
    pros::delay(1250);
    move(0,0);

    pros::delay(100000000); //temp

    chassis.turnTo(-52.65,-27.17, 1000, false, 80);

    chassis.moveTo(-52.65, -27.17, 2000, 80);

    chassis.turnTo(-57,-23, 1000, false, 70);

    move(127, 0);
    pros::delay(500);
    frontwings.set_value(true);
    pros::delay(500);
    move(0, 0);

    move(80, 20);
    pros::delay(1000);
    move(-50,0);
    pros::delay(800);
    move(127,20);
    pros::delay(750);
    move(-75, 0);
    pros::delay(800);
    move(127,30);
    pros::delay(1000);
    move(0,0);


    
}
void left_auton() {
    wings.set_value(true);
    pros::delay(250);
    chassis.moveTo(0, -15, 2000, 127);
    wings.set_value(false);
    pros::delay(250);
    chassis.turnTo(5,-22.5, 2000, true, 127);
    chassis.moveTo(5,-22.5, 2000, 127);
    intake.move(127);
    move(-127, 0);
    pros::delay(250);
    intake.move(-127);
    move(70, 0);
    pros::delay(175);
    intake.move(127);
    move(-127, 0);
    pros::delay(250);
    intake.move(-127);
    move(0, 0);
    pros::delay(250);
    intake.move(0);
    chassis.turnTo(3.2, -6.6, 2000, false, 127);
    chassis.moveTo(3.2,-6.6,2000 ,127);
    intake.move(127);
    chassis.turnTo(35.6, 1.5, 2000, false, 127);
    chassis.moveTo(35.6, 1.5,2000 ,127);

    chassis.turnTo(42.3, -6.9, 2000, false, 127);
    chassis.moveTo(42.3, -6.9,2000 ,127);

    chassis.turnTo(26.7, -23.4, 2000, false, 127);

    pros::delay(500);

    frontwings.set_value(true);

    intake.move(0);

    pros::delay(500);

    chassis.moveTo(26.7, -23.4,1000 ,127);

    pros::delay(250);

    move_drive(-50,0);

    pros::delay(100);

    frontwings.set_value(false);

    pros::delay(100);

    move_drive(0,0);

    pros::delay(150);

    move_drive(127,0);

    pros::delay(600);

    move_drive(-127,0);

    pros::delay(200);

    move_drive(0,0);

}

void right_auton() {
    wings.set_value(true);

    pros::delay(250);
    chassis.moveTo(0, -12.75, 2000, 70); // move to the point (53, 53) with a timeout of 1000 ms
    
    wings.set_value(false);

    pros::delay(1000);

    chassis.moveTo(0, -20, 2000, 70); // move to the point (53, 53) with a timeout of 1000 ms
    chassis.turnTo(-6.7, -25.5, 2000, true, 70);
    chassis.moveTo(-6.7, -25.5, 750, 70);
    
    chassis.moveTo(-4.5,-22, 750, 127);
    pros::delay(200);
    chassis.moveTo(-10, -32, 750, 127);
    pros::delay(200);
    chassis.moveTo(-4.5,-22, 750, 127);
    pros::delay(200);
    chassis.moveTo(-10, -32, 750, 127);
    pros::delay(200);

    chassis.moveTo(-3,-17, 1000, 70);

    chassis.turnTo(-4.1, 1.8, 2000, false, 70);
    chassis.moveTo(-4.1, 1.8, 2000, 70);

    move(60,0);
    pros::delay(140);
    move(0,0);
    

    chassis.turnTo(-19.6, 19.5, 2000, false, 70);
    chassis.moveTo(-19.6, 19.5, 2000, 70);
}

void autonomous() {
    skills();
    

    // right_auton();
    // left_auton();
    // skills();

}

void opcontrol() {
    bool block_activate = false;
    bool wings_activate = false;
    bool frontwings_activate = false;

	while (true) {

        int power = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);





        printf("P: %d T: %d \n", power, turn);


        move_drive(power, dampen(turn));

        bool outtakebutton = master.get_digital(DIGITAL_R2);
        bool intakebutton = master.get_digital(DIGITAL_R1);

        if (intakebutton){
            intake = 127;
        }
        else if (outtakebutton){
            intake = -127;
        }
        else{
            intake = 0;
        }

        bool frontbutton = master.get_digital_new_press(DIGITAL_L1);
        bool wingsbutton = master.get_digital_new_press(DIGITAL_L2);

        if (wingsbutton && !wings_activate){
            wings.set_value(true);
            wings_activate = true;
        }
        else if (wingsbutton && wings_activate){
            wings.set_value(false);
            wings_activate = false;
        }

        if (frontbutton && !frontwings_activate){
            frontwings.set_value(true);
            frontwings_activate = true;
        }
        else if (frontbutton && frontwings_activate){
            frontwings.set_value(false);
            frontwings_activate = false;
        }

        bool cata = master.get_digital(DIGITAL_LEFT);
        if(cata) {
            catapult.move(127);
        } else {
            catapult.move(0);
        }

        pros::delay(20);
	}
}
