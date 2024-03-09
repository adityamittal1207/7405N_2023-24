#include "main.h"
#include "lemlib/api.hpp"
#include <functional>

ASSET(path_txt);
ASSET(path2_txt);
ASSET(path3_txt);
ASSET(path4_txt);
ASSET(path5_txt);
ASSET(path6_txt);
ASSET(path7_txt);
ASSET(path8_txt);
ASSET(path9_txt);
ASSET(path10_txt);

pros::Motor left_front_motor(15, pros::E_MOTOR_GEARSET_06, true);
pros::Motor left_center_motor(14, pros::E_MOTOR_GEARSET_06, true);
pros::Motor left_back_motor(10, pros::E_MOTOR_GEARSET_06, true);
pros::Motor right_front_motor(7, pros::E_MOTOR_GEARSET_06, false);
pros::Motor right_center_motor(6, pros::E_MOTOR_GEARSET_06, false);
pros::Motor right_back_motor(9, pros::E_MOTOR_GEARSET_06, false);

pros::Motor intake(20); //
pros::Motor catapult(4);
pros::Motor catapult2(21, true);


pros::Imu inertial_sensor(3); // port 8


pros::ADIDigitalOut backLeftWing('G'); //backwings H, F
pros::ADIDigitalOut backRightWing('D'); 
pros::ADIDigitalOut frontLeftWing('F'); //backwings H, F
pros::ADIDigitalOut frontRightWing('H'); 
pros::ADIDigitalOut hang('E');

pros::MotorGroup left_side_motors({left_front_motor, left_center_motor, left_back_motor});
pros::MotorGroup right_side_motors({right_front_motor, right_center_motor, right_back_motor});

pros::Controller master(pros::E_CONTROLLER_MASTER);

lemlib::Drivetrain drivetrain {
        &left_side_motors, // left drivetrain motors
        &right_side_motors, // right drivetrain motors
        10, // track width
        3.25, // wheel diameter
        360, // wheel rpm
        8
};

pros::Rotation vertical_rot(16, true); // port 1, not reversed
pros::Rotation horizontal_rot(11, false); // port 1, not reversed

lemlib::TrackingWheel vertical_track(&vertical_rot, 2, -3.2);
lemlib::TrackingWheel horizontal_track(&horizontal_rot, 2, -6.5); // 0.6 -0.9


// odometry struct
lemlib::OdomSensors sensors {
        &vertical_track, // vertical tracking wheel 1
        nullptr, // vertical tracking wheel 2
        // nullptr,
        &horizontal_track, // horizontal tracking wheel 1
        nullptr, // we don't have a second tracking wheel, so we set it to nullptr
        &inertial_sensor // inertial sensor
};

// forward/backward PID
lemlib::ControllerSettings lateralController {
        7.5, // kP
        0,
        10, // kD
        0,
        1, // smallErrorRange
        100, // smallErrorTimeout
        3, // largeErrorRange
        500, // largeErrorTimeout
        5 // slew rate
};

// turning PID
lemlib::ControllerSettings angularController {
        5, // kP
        0,
        40, // kD
        0,
        1, // smallErrorRange
        100, // smallErrorTimeout
        3, // largeErrorRange
        500, // largeErrorTimeout
        40 // slew rate
};

lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

double shooter_coeff = 0.775;


void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void move(double power, double turn, bool swing=false) {
    int left = power + turn;
    int right = power - turn;

    if (swing && left < 0) {left = 0;}
    if (swing && right < 0) {right = 0;}

    left_front_motor = left;
    left_center_motor = left;
    left_back_motor = left;
    right_front_motor = right;
    right_center_motor = right;
    right_back_motor = right;
}

void move_drive(double power, double turn) {

    pros::lcd::print(2, "turnpower: %f", turn);

    int left = power + turn;
    int right = power - turn;

    if (left < 0) left = left - 9;
    else if (left > 0) left = left + 9;

    if (right < 0) right = right - 9;
    else if (right > 0) right = right + 9;


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

        // printf("x: %f | y: %f | H: %f | rot: %d \n", pose.x, pose.y, pose.theta, vertical_rot.get_position());
        pros::delay(10);
    }
}
static int dampen(int input) {
    /*
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
    */
    double expo = 0.4;
    double output = 0;
    double in = input / 127.0;
    output = ((in * in * in) * 118 * expo) + in * 118 * (1 - expo);
    return (int)std::round(output);
}
void rotate_to(double targetHeading, double turnAcc, double maxSpeed, bool swing) {
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

        turnSpeed = headingErr * 1.2 + errorsum * 0.01;

        // if (i % 3 == 0) {
        //     std::cout << "curPos: " << curPos.toString() << ", targetHeading: " << targetHeading << ", "
        //               << "turnCompleteBuff: " << turnCompleteBuff << ", "
        //               << "headingErr: " << headingErr << ", turnSpeed: " << turnSpeed
        //               << std::endl;
        //     printf("turning turn function\n");
        // }

        if(std::abs(turnSpeed) > maxSpeed) {turnSpeed = turnSpeed < 0 ? -maxSpeed : maxSpeed; }

        move(0, turnSpeed, swing);
    }

    printf("turn done");
    move(0, 0);
}


void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    vertical_rot.reset_position();
    horizontal_rot.reset_position();
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


void hang_test() {
    hang.set_value(true);
    move(-80,0);
    pros::delay(575);
    catapult.move(90);
    catapult2.move(90);
    pros::delay(425);
    catapult.move(0);
    catapult2.move(0);
    move(0,0);
    backRightWing.set_value(true);
    pros::delay(100);
    hang.set_value(false);
    catapult.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    catapult2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    catapult.brake();
    catapult2.brake();
    pros::delay(1000);
    backRightWing.set_value(false);

}

void skills() {
    chassis.follow(path7_txt, 15, 2000, false, true);

    pros::delay(1500);
    
    move(60, 0);
    pros::delay(100);
    move(-127, 0);
    pros::delay(300);
    move(0, 0);

    chassis.moveToPoint(2, -22.2, 2000, true, 127);

    chassis.turnTo(-16.3,-14.5, 2000, true, 100);

    backRightWing.set_value(true);

    pros::delay(500);

    // pros::delay(75000000000000);

    catapult.move(127 * 0.715);
    catapult2.move(127 * 0.715);

    // Activate cata

    pros::delay(27000);
    // pros::delay(1500);

    // 0.8,-22.5,-3.5
    // -7.6,19.4,-34.1
    // -31.5,53.3,-34.1
    // -78.3,63.4,-1.3

    // pros::delay(888989898998899090);

    catapult.move(0);
    catapult2.move(0);

    backRightWing.set_value(false);
    pros::delay(500);

    chassis.turnTo(0, 1, 2000, true, 100);


    // pros::del
    chassis.follow(path8_txt, 15, 5700, true);


    // chassis.waitUntilDone();

    // chassis.follow(path9_txt,15,5000,true);

    // -49.8,55.7,-41

    // -90.5,54.9,-126
    // pros::delay(2700);
    // frontLeftWing.set_value(true);

    chassis.waitUntilDone();
    move(-50,0);
    pros::delay(300);
    move(127,-10);
    pros::delay(400);
    move(-100,0);
    pros::delay(300);
    move(110,0);
    pros::delay(300);
    move(0,0);

    // pros::delay(393939393939393939333);

    // frontLeftWing.set_value(false);

    // pros::delay(50004305053405430543503530);

    chassis.moveToPoint(-79,61.5, 500, false, 127);

    // pros::delay(3924934039324924324092342034324324);

    chassis.turnTo(-69.7,9.6, 500, true, 100);

    chassis.moveToPoint(-62.9,14, 1300, true, 100);

    // pros::delay(3943299434394359454353545345945);

    chassis.turnTo(-89.8,30, 2000, false, 100);

    // pros::delay(93457349534753497523455);

    backLeftWing.set_value(true);

    chassis.moveToPoint(-93,24.6, 700, false, 90);

    // pros::delay(93457349534753497523455);

    chassis.waitUntilDone();

    move(0,40);

    pros::delay(200);

    move(-90,0);

    // move(90,20);

    pros::delay(500);

    move(0, 0);

    // pros::delay(39429423943943294239439429493249234);
// -80.8,-1.6,50

    backLeftWing.set_value(false);

    chassis.moveToPoint(-76.9, 1.8, 2000, true);

    // pros::delay(93457349534753497523455); 

    // -76.9, 1.8

    chassis.turnTo(-86.0,-6.4, 2000, false, 100);

    chassis.moveToPoint(-86.0,-6.4, 2000, false);

    chassis.turnTo(-105.3,16.8, 2000, false, 100);

    pros::delay(500);

    // pros::delay(394293423423424234);

    // backLeftWing.set_value(true);
    backLeftWing.set_value(true);
    backRightWing.set_value(true);

    chassis.moveToPoint(-105.3,16.8, 2000, false, 90);


    chassis.waitUntilDone();

    move(-90,0);

    pros::delay(350);

    move(80,0);

    pros::delay(300);

    move(-100,0);

    pros::delay(500);

    move(0,0);

    backLeftWing.set_value(false);
    backRightWing.set_value(false);

    chassis.moveToPoint(-88.2, -4.8, 2000, true, 127);

    // pros::delay(129234949329423947);

    // pros::delay(999999999999999999);

// clipped from 422-468

    // chassis.turnTo(-115.4,-19.3, 2000, false, 100);

    // // // pros::delay(239493432943294329294);

    // chassis.moveToPoint(-115.4,-19.3, 2000, false, 127);

    // // backLeftWing.set_value(true);

    // chassis.turnTo(-116.2, 2.5, 2000, false, 100);

    // backRightWing.set_value(true);

    // backLeftWing.set_value(true);

    // chassis.moveToPoint(-116.2, 2.5, 1500, false, 127);

    // chassis.waitUntilDone();

    // // backLeftWing.set_value(false);

    // // backRightWing.set_value(false);

    // move(0,-40);

    // pros::delay(300);

    // move(-127,0);

    // pros::delay(300);

    // move(0,0);

    // backLeftWing.set_value(false);
    // backRightWing.set_value(false);

    // chassis.turnTo(-91.8, -11.9, 2000, true, 100);

    // chassis.moveToPoint(-91.8, -11.9, 2000, true, 127);

    // // front turn move 

    // // pros::delay(100000000000000000000);

    // backLeftWing.set_value(false);
    // backRightWing.set_value(false);

    // pros::delay(100000000000000000000);

    // -129,-2, -270 heading

    chassis.turnTo(-129,-2, 2000, false, 100);

    // backLeftWing.set_value(true);

    chassis.moveToPoint(-129,-2, 2000, false, 100);

    //  back turn move

    // moveto -93.1, -13

    // chassis.moveToPoint(-99.4, -11, 1500, false, 127);

    // backLeftWing.set_value(false);

    // -125.6, -7.3, -270

    // -150,-15.3,-283

    // -154.6,11.2,-133

    // chassis.setPose(-125.6, -7.3, -270);

    // pros::delay(100009876543212345678234567);

    chassis.follow(path10_txt, 12, 2500, false, true);


    // pros::delay(500394239432942392423934932994239329);

    // chassis.turnTo(-128.7, -12.5, 2000, false, 100);

    // chassis.moveToPoint(-128.7, -12.5, 2000, false, 127);

    chassis.waitUntilDone();

    move(0, 30);

    pros::delay(350);

    move(-127,0);

    pros::delay(300);

    move(80,0);

    pros::delay(300);

    move(-127,0);

    pros::delay(300);

    move(0,0);  

// -145.5, 26.2 

    // chassis.turnTo(-142.0, -23.4, false, 100);

    // // backLeftWing.set_value(true);

    // chassis.moveToPoint(-142.0, -23.4, 2000, false, 100);

    // chassis.turnTo(-137.5,-30.19, 2000, false, 100);

    // hang_test();

//  turn back, move back 
//  back turn 
    // // frontLeftWing.set_value(true);

    // chassis.moveToPoint(-112.3, 7, 1000, false, 127);

    // chassis.waitUntilDone();

    // move(-127, -20);

    // pros::delay(400);

    // move(0,0);

    // move(-127,0);

    // pros::delay(400);

    // move(0,0);


}


void left_auton() {
    backLeftWing.set_value(true);

    intake.move(127);
    chassis.moveToPoint(0, 4, 2000, true, 127);

    backLeftWing.set_value(false);

    chassis.follow(path6_txt, 15, 2000, false, true);
    pros::delay(750);
    backLeftWing.set_value(true);
    pros::delay(500);
    backLeftWing.set_value(false);

    chassis.moveToPoint(19, -56, 1000, true, 127);

    chassis.turnTo(24, -31, 300, true, 127);

    chassis.turnTo(60, -58, 300, true, 127);

    chassis.waitUntilDone();

    intake.move(-127);


    move(127, 0);
    pros::delay(400);
    move(0, 0);
    // pros::delay(200);
    // move(0, 0);

    pros::delay(10000000000);
    chassis.moveToPoint(18, -56, 750, false, 127);

    chassis.turnTo(34, 0, 750, true, 100);

    intake.move(127);


    chassis.moveToPoint(34, 0, 1500, true, 127);

    chassis.moveToPoint(31.5, -14.0, 1500, false, 127);

    chassis.turnTo(42.1, -24.2, 1000, true, 100);


    chassis.moveToPoint(35, -19.5, 1500, true, 127, true);
        intake.move(-60);

        pros::delay(300);



    chassis.turnTo(54, -1, 1000, true, 100);
    intake.move(100);


    chassis.moveToPoint(54, -1, 1000, true, 127);


    chassis.turnTo(53.7, -33.3, 2000, true, 100);

    frontLeftWing.set_value(true);
    frontRightWing.set_value(true);

    chassis.moveToPoint(53.7, -33.3, 1000, true, 127);

    chassis.waitUntilDone();

    move(127, 0);
    pros::delay(500);
    move(-127, 0);
    pros::delay(300);
    frontLeftWing.set_value(false);
    frontRightWing.set_value(false);
    move(0, 0);


    // pros::delay(250);

    // intake.move(-127);
    // pros::delay(500);
}

void right_auton() {
    frontLeftWing.set_value(true);
    backLeftWing.set_value(true);

    intake.move(127);
    chassis.moveToPoint(0, 40, 2000, true, 127, true);
    pros::delay(250);
    backLeftWing.set_value(false);
    pros::delay(250);
    frontLeftWing.set_value(false);
    pros::delay(1000);

    chassis.moveToPoint(0, 48, 2000, false, 127); 

    chassis.turnTo(18, 54, 2000, true, 127);

    frontLeftWing.set_value(true);
    frontRightWing.set_value(true);

    chassis.moveToPoint(18, 54, 1500, true, 127);
    intake.move(-127);    
    chassis.moveToPoint(6, 47, 1000, false, 127);

    frontLeftWing.set_value(false);
    frontRightWing.set_value(false);

    chassis.turnTo(-15, 9, 500, false, 100);

    chassis.moveToPoint(-15, 9, 2000, false, 127);

    // pros::delay(5000000000000000);

    // , move fortward

// -9.3, -2.9, back

    chassis.turnTo(-7.8,-2.7 , 500, false, 100);

    pros::delay(500);

    backLeftWing.set_value(true);

    pros::delay(500);

    // pros::delay(3994493294399432434324242);

    // chassis.turnTo(-16.3,4.8, 700, false, 100);

    chassis.moveToPoint(-15,5.3 , 600, false, 127);

    chassis.waitUntilDone();

    move(0,-127);

    pros::delay(200);

    move(0,0);

    // -9.8 -0.6 

    // pros::delay(39494439242943293424324234);

    // move(-60,0);

    // pros::delay(100);9.10, -1.3

    // move(0,0);

    chassis.turnTo(-22.6, 12.1, 2000, false, 100);

    backLeftWing.set_value(false);


    chassis.follow(path_txt, 15, 1000, false, true);

    pros::delay(600);
    
    move(-127, 0);
    pros::delay(300);
    move(0, 0);


    chassis.moveToPoint(-25.6, 16.2, 1250, true, 127);
    
    // pros::delay(223842834283482342834238428348324234);

     chassis.turnTo(-3,-2.2, 750, true, 100);  
    chassis.moveToPoint(-3,-2.2, 1000, true, 127);

    // 25.4,1.2

    chassis.turnTo(25.4,1.2, 750, true, 100);  
    chassis.moveToPoint(25.4,1.2, 1000, true, 127);    
    
}

void weirdright() {
    backRightWing.set_value(true);
    // backrightdownhg
    pros::delay(300);
    chassis.moveToPoint(0, -17.5, 2500, false, 127);  

    pros::delay(500);
    // move back to 
    backRightWing.set_value(false);
    chassis.moveToPoint(0, 0, 1000, true, 127);  
    // turn move 
    chassis.turnTo(-23.34,24.9, 2500, true, 80);  
    chassis.moveToPoint(-23.34,24.9, 2500, true, 80);    
    // backrightup 
    // move forward to 0,0

}

void autonomous() {
    // skills();
    // left_auton();
    right_auton();
    // weirdright();
    // hang_test();
}


void opcontrol() {
    bool block_activate = false;
    bool wings_activate = false;
    bool hang_activate = false;
    bool _activate = false;
    int hang_step = 0;
    bool macron = false;

    bool cata_activate = false;
    double max_coeff = 0.9;

    int counter = 0;

    bool leftBackWingActivate = false;
    int leftBackWingCount = 0;
    bool leftBackWingAct = false;
    bool yesjig = true;
    bool jig_hang_activate = true;



	while (true) {

        int power = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);
        move_drive(power, dampen(turn));
        

        bool outtakebutton = master.get_digital(DIGITAL_R2);
        bool intakebutton = master.get_digital(DIGITAL_R1);


        bool frontbutton = master.get_digital_new_press(DIGITAL_L1);
        bool wingsbutton = master.get_digital_new_press(DIGITAL_L2);

        bool frontbutton_dig = master.get_digital(DIGITAL_L1);
        bool wingsbutton_dig = master.get_digital(DIGITAL_L2);



        if(frontbutton_dig && wingsbutton_dig && leftBackWingCount % 2 == 0) {
            leftBackWingActivate = true; 
            leftBackWingCount++;
        } else if (frontbutton_dig && wingsbutton_dig && leftBackWingCount % 2 != 0){
            leftBackWingActivate = false;
        } else if (!frontbutton_dig && !wingsbutton_dig && leftBackWingCount % 2 != 0){
            leftBackWingActivate = false;
            leftBackWingCount++;
        }

        printf("Activate: %d \n", leftBackWingActivate);


        if(leftBackWingActivate && !leftBackWingAct) {
            backLeftWing.set_value(true);
            leftBackWingAct = true;
        } else if (leftBackWingActivate && leftBackWingAct) {
            backLeftWing.set_value(false);
            leftBackWingAct = false;
        }


        if (wingsbutton && !wings_activate && !_activate && !leftBackWingActivate){
            backLeftWing.set_value(true);
            backRightWing.set_value(true);
            wings_activate = true;
        }
        else if (wingsbutton && wings_activate){
            backLeftWing.set_value(false);
            backRightWing.set_value(false);
            wings_activate = false;
        }

        if (frontbutton && !_activate && !wings_activate && !leftBackWingActivate){
            frontLeftWing.set_value(true);
            frontRightWing.set_value(true);
            _activate = true;
        } 
        else if (frontbutton && _activate){
            frontLeftWing.set_value(false);
            frontRightWing.set_value(false);
            _activate = false;
        }

        if (intakebutton){
            intake = 127;
        }
        else if (outtakebutton){
            intake = -127;
        }
        else{
            intake = 0;
        }


        bool cata = master.get_digital_new_press(DIGITAL_LEFT) || master.get_digital_new_press(DIGITAL_UP);
        if(cata && !cata_activate) {
            catapult.move(127 * shooter_coeff);
            catapult2.move(127 * shooter_coeff);
            cata_activate = true;

        } else if(cata && cata_activate) {
            catapult.move(0);
            catapult2.move(0);
            cata_activate = false;
            if (macron){
                macron = false;
                backRightWing.set_value(false);
                wings_activate = false;
            }
        }
        // bool jigcata = master.get_digital_new_press(DIGITAL_Y);
        bool jigcata = false;

        if(jigcata) {
            catapult.move(50);
            catapult2.move(50);
            jig_hang_activate = true;
            pros::delay(500);
            catapult.move(10);
            catapult2.move(10);
        }
        // } else if (jigcata && jig_hang_activate) {
        //     catapult.move(10);
        //     catapult2.move(10);
        //     jig_hang_activate = false;
        // }

        bool hang_ = master.get_digital_new_press(DIGITAL_X);

    //     if(hang_ && !hang_activate) {
    //         hang.set_value(true);
    //         hang_activate = true;
    //     } else if (hang_ && hang_activate) {
    //         hang.set_value(false);
    //         hang_activate = false;
    //     }

    //     hang up

    // wing down 

    // pros::delay(100);

    // hang down 

    // pros::delay(1000);

    // wing up 
        
        if (hang_ && hang_step == 0){
            hang.set_value(true);
            hang_activate = true;
            hang_step += 1;
        }
        else if (hang_ && hang_step == 1){
            backRightWing.set_value(true);
            pros::delay(100);
            hang.set_value(false);
            pros::delay(1000);
            backRightWing.set_value(false);
        }

        // bool macrobutton_ = master.get_digital_new_press(DIGITAL_B);

        bool macrobutton_ = false;

        if (macrobutton_){
            printf("macro pressed");

            macron = true;

            chassis.follow(path7_txt, 15, 2000, false, true);

            backLeftWing.set_value(true);
            pros::delay(100);
            backLeftWing.set_value(false);

            // pros::delay(399932439234234293432432432423);

            pros::delay(1500);
            
            move(60, 0);
            pros::delay(100);

            // move(0,0);
            // pros::delay(92434934324243242423432423);

            move(-127, 0);
            pros::delay(300);
            move(0, 0);

            // pros::delay(39483294238492382934234234234234);

            chassis.moveToPoint(0.2, -24, 2000, true, 127);

            // pros::delay(39483294238492382934234234234234);

            chassis.turnTo(-10.2, -18.8, 2000, true, 100);

            chassis.moveToPoint(1.9, -25.2, 2000, true, 100);
            
            backRightWing.set_value(true);

            wings_activate = true;

            pros::delay(300);

            catapult.move(127 * shooter_coeff);
            catapult2.move(127 * shooter_coeff);
            cata_activate = true;
        }

        pros::delay(20);
	}
}
