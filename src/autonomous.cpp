// #include "main.h"
// #include "lib/Robot.h"

// void autonomous() {

// //    left_side();
//     right_side();

// }

// void skills() {
// //    Robot::flywheel.set_velocity(1925);

//     Robot::drive.move(-40, 0);
//     Robot::INT = 100;

//     pros::delay(650);
//     Robot::INT = 127;


//     Robot::power.set_value(12, 0.01, 0, 2, 10);
//     Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//     Robot::drive.move_to(Pose(0, 4), 1, 10000, 127, 1000); // 0, -5

//     Robot::INT = 127;

//     pros::delay(500);

//     Robot::turn.set_value(2.5, 0.001, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(-32, 1);

//     //
//     Robot::power.set_value(7.2, 0.01, 0, 2, 10);
//     Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//     Robot::drive.move_to(Pose(-13, 21), 3, 10000); // 0, -5

//     pros::delay(500);

//     Robot::turn.set_value(1, 0.001, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(90, 0.5);

//     Robot::power.set_value(7.2, 0.01, 0, 2, 10);
//     Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//     Robot::drive.move_to(Pose(-20, 21), 3, 10000); // 0, -5

//     Robot::drive.move(-40, 0);
//     Robot::INT = 100;

//     pros::delay(450);
//     Robot::INT = 127;
//     pros::delay(450);


//     Robot::power.set_value(7.2, 0.01, 0, 2, 10);
//     Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//     Robot::drive.move_to(Pose(-18, 21), 3, 10000); // 0, -5

//     pros::delay(250);

//     Robot::turn.set_value(1.3, 0.0008, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(1.5, 0.5);

//     Robot::flywheel.set_velocity(1630);

//     pros::delay(250);

//     Robot::power.set_value(7.2, 0.01, 0, 2, 10);
//     Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//     Robot::drive.move_to(Pose(-19, 77), 3, 10000, 80); // 0, -5

//     pros::delay(100);

//     Robot::turn.set_value(5, 0.001, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(6.5, 0.5);

//     pros::delay(250);

//     Robot::INT = -127;
//     pros::delay(400);
//     Robot::INT = 0;
//     pros::delay(600);

//     Robot::INT = -127;
//     pros::delay(400);
//     Robot::INT = 0;
//     pros::delay(600);

//     Robot::INT = -127;
//     pros::delay(400);
//     Robot::INT = 0;
//     pros::delay(600);

//     pros::delay(500);

//     Robot::turn.set_value(3, 0.0008, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(0, 0.5);

//     pros::delay(250);

//     Robot::power.set_value(7.2, 0.01, 0, 2, 10);
//     Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//     Robot::drive.move_to(Pose(-18, 34), 5, 2000, 80, 2000); // 0, -5

//     pros::delay(250);

//     Robot::turn.set_value(1.5, 0.001, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(46, 0.5);

//     Robot::INT = 127;

//     pros::delay(250);

//     Robot::power.set_value(7.2, 0.01, 0, 2, 10);
//     Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//     Robot::drive.move_to(Pose(27, 73.5), 3, 10000, 80); // 0, -5

//     pros::delay(250);

//     Robot::turn.set_value(1.2, 0.001, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(-41.5, 0.5);

//     pros::delay(250);

//     Robot::INT = -127;
//     pros::delay(400);
//     Robot::INT = 0;
//     pros::delay(600);

//     Robot::INT = -127;
//     pros::delay(400);
//     Robot::INT = 0;
//     pros::delay(600);

//     Robot::INT = -127;
//     pros::delay(400);
//     Robot::INT = 0;
//     pros::delay(600);

//     pros::delay(250);

//     Robot::turn.set_value(1.2, 0.001, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(43, 0.5);

//     Robot::INT = 127;

//     pros::delay(250);

//     Robot::power.set_value(7.2, 0.01, 0, 2, 10);
//     Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//     Robot::drive.move_to(Pose(59, 114.5), 3, 10000, 100, 1000); // 0, -5

//     pros::delay(250);

//     Robot::power.set_value(7.2, 0.01, 0, 2, 10);
//     Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//     Robot::drive.move_to(Pose(69.4, 118.3), 3, 10000, 80, 2800); // 0, -5

//     pros::delay(250);

//     Robot::turn.set_value(1.25, 0.001, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(-92, 0.5);

//     pros::delay(250);

//     Robot::power.set_value(7.2, 0.01, 0, 2, 10);
//     Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//     Robot::drive.move_to(Pose(29.6, 118.9), 3, 10000, 80, 2800); // 0, -5

//     pros::delay(250);


//     Robot::turn.set_value(5, 0.001, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(-94, 0.5);

//     pros::delay(250);


//     Robot::INT = -127;
//     pros::delay(400);
//     Robot::INT = 0;
//     pros::delay(600);

//     Robot::INT = -127;
//     pros::delay(400);
//     Robot::INT = 0;
//     pros::delay(600);

//     Robot::INT = -127;
//     pros::delay(400);
//     Robot::INT = 0;
//     pros::delay(600);

//     pros::delay(500);

//     Robot::turn.set_value(5, 0.001, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(-92, 0.5);

//     pros::delay(500);

//     Robot::power.set_value(7.2, 0.01, 0, 2, 10);
//     Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//     Robot::drive.move_to(Pose(83.6, 117.88), 3, 10000, 80, 2500); // 0, -5

//     pros::delay(250);

//     Robot::turn.set_value(1.2, 0.001, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(-180, 0.5);

//     pros::delay(250);

//     Robot::drive.move(-40, 0);
//     Robot::INT = 100;

//     pros::delay(450);
//     Robot::INT = 127;
//     pros::delay(650);

//     Robot::drive.move(40, 0);
//     pros::delay(650);
//     Robot::drive.move(0, 0);

//     Robot::turn.set_value(5, 0.001, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(-172.6, 0.5);

//     pros::delay(250);

//     Robot::power.set_value(7.2, 0.01, 0, 2, 10);
//     Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//     Robot::drive.move_to(Pose(78.2, 106.3), 3, 10000, 90, 1400); // 0, -5

//     pros::delay(250);

//     Robot::power.set_value(7.2, 0.01, 0, 2, 10);
//     Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//     Robot::drive.move_to(Pose(72.67, 87.6), 3, 10000, 80, 2800); // 0, -5

//     pros::delay(250);

//     Robot::turn.set_value(1.7, 0.001, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(-128, 0.5);

//     pros::delay(250);

//     Robot::power.set_value(7.2, 0.01, 0, 2, 10);
//     Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//     Robot::drive.move_to(Pose(98.9, 106.5), 3, 10000, 80, 2800); // 0, -5

//     pros::delay(250);

//     Robot::turn.set_value(1.7, 0.001, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(-90, 0.5);

//     pros::delay(250);

//     Robot::drive.move(-40, 0);
//     Robot::INT = 100;

//     pros::delay(450);
//     Robot::INT = 127;
//     pros::delay(450);

//     Robot::drive.move(40, 0);
//     pros::delay(350);
//     Robot::drive.move(0, 0);

//     pros::delay(250);

//     Robot::turn.set_value(1.25, 0.001, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(-180, 0.5);

//     pros::delay(250);

//     Robot::power.set_value(7.2, 0.01, 0, 2, 10);
//     Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//     Robot::drive.move_to(Pose(104.2, 50.6), 3, 10000, 80, 2300); // 0, -5

//     Robot::turn.set_value(5, 0.001, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(-174, 0.5);

//     pros::delay(250);


//     Robot::INT = -127;
//     pros::delay(400);
//     Robot::INT = 0;
//     pros::delay(600);

//     Robot::INT = -127;
//     pros::delay(400);
//     Robot::INT = 0;
//     pros::delay(600);

//     Robot::INT = -127;
//     pros::delay(400);
//     Robot::INT = 0;
//     pros::delay(600);

//     pros::delay(500);

//     Robot::turn.set_value(5, 0.001, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(-180, 0.5);

//     pros::delay(250);

//     Robot::power.set_value(7.2, 0.01, 0, 2, 10);
//     Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//     Robot::drive.move_to(Pose(99.4, 116.5), 3, 10000, 127, 2200); // 0, -5

//     pros::delay(250);

//     Robot::turn.set_value(1.5, 0.001, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(-135, 0.5);

//     Robot::EXP.set_value(true);


// //
// //    Robot::drive.move_to(Pose(91.24, 119.33), 7, 10000, 127, 3000); // 0, -5
// //
// //    pros::delay(500);
// //
// //    Robot::turn.set_value(1.5, 0.002, 0.0, 6, 0.0);
// //    Robot::drive.rotate_to(-145, 5);
// //
// //    pros::delay(500);
// //    Robot::EXP.set_value(true);

// }

// void right_side() {

//     Robot::flywheel.set_velocity(1720);
//     Robot::power.set_value(7.2 , 0.01, 0, 3, 10);
//     Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//     Robot::drive.move_to(Pose(0.0, -21), 3, 100000, 80);

//     pros::delay(500);

//     Robot::turn.set_value(1.25, 0.0008, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(90, 0.5);

//     pros::delay(250);

//     Robot::drive.move(-40, 0);
//     Robot::INT = 100;

//     pros::delay(500);
//     Robot::INT = 127;

//     Robot::drive.move_to(Pose(3, -21), 3, 100000, 100);

//     pros::delay(250);

//     // Robot::turn.set_value(1.25, 0.001, 0.0, 6, 0.0);
//     // Robot::drive.rotate_to(98, 0.5);

//     // pros::delay(1000);


//     // while(!Robot::flywheel.reached_target(5)) pros::delay(5);

//     // Robot::INT = -127;
//     // pros::delay(400);
//     // Robot::INT = 0;


//     // while(!Robot::flywheel.reached_target(5)) pros::delay(5);

//     // Robot::INT = -127;
//     // pros::delay(400);
//     // Robot::INT = 0;


//     Robot::turn.set_value(1.4, 0.0008, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(45.65, 0.5);

//     Robot::INT = 127;


//     pros::delay(250);

//     Robot::power.set_value(7.2 , 0.01, 0, 3, 10);
//     Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//     Robot::drive.move_to(Pose(21.89, 3.63), 3, 100000, 90);

//     pros::delay(260);

//     Robot::turn.set_value(1.275, 0.0008, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(107.3, 0.5);

//     pros::delay(800);

//     // while(!Robot::flywheel.reached_target(10)) pros::delay(5);

//      Robot::INT = -127;
//      pros::delay(365);
//      Robot::INT = 0;
//      pros::delay(300);

//     // while(!Robot::flywheel.reached_target(10)) pros::delay(5);

//     Robot::INT = -127;
//      pros::delay(365);
//      Robot::INT = 0;
//      pros::delay(300);

//     Robot::flywheel.set_velocity(1780);
//     // while(!Robot::flywheel.reached_target(10)) pros::delay(5);

//     Robot::INT = -127;
//      pros::delay(365);
//      Robot::INT = 127;
//      pros::delay(300);

//     Robot::flywheel.set_velocity(1713.2);

//     Robot::turn.set_value(1.275, 0.0008, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(41.8, 0.5);

//     pros::delay(250);

//     Robot::power.set_value(7.2 , 0.01, 0, 3, 10);
//     Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//     Robot::drive.move_to(Pose(49, 33), 3, 100000, 90);

//     pros::delay(500);

//     Robot::turn.set_value(1.25, 0.0008, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(134, 0.5);

//     pros::delay(800);

//     while(!Robot::flywheel.reached_target(5)) pros::delay(5);

//     Robot::INT = -127;
//     pros::delay(400);
//     Robot::INT = 0;


//     while(!Robot::flywheel.reached_target(5)) pros::delay(5);

//     Robot::INT = -127;
//     pros::delay(400);
//     Robot::INT = 0;
// }

// void left_side() {

//     Robot::flywheel.set_velocity(1900);

//     Robot::drive.move(-40, 0);
//     Robot::INT = 100;

//     pros::delay(250);
//     Robot::INT = 127;


//     // Robot::power.set_value(12, 0.01, 0, 2, 10);
//     // Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//     // Robot::drive.move_to(Pose(0, 4), 1, 10000, 127, 1000); // 0, -5

//     // Robot::INT = 127;  

//     // pros::delay(500);

// //     Robot::turn.set_value(2.5, 0.001, 0.0, 6, 0.0);
// //     Robot::drive.rotate_to(-32, 1);
// // //
// //     Robot::power.set_value(7.2, 0.01, 0, 2, 10);
// //     Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
// //     Robot::drive.move_to(Pose(-5.7, 10.3), 3, 10000); // 0, -5

// //     pros::delay(500);
//     Robot::power.set_value(7.2, 0.01, 0, 2, 10);
//     Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//     Robot::drive.move_to(Pose(-0.23, 3.63), 3, 10000); // 0, -5
    
//     pros::delay(250);

//     Robot::turn.set_value(3, 0.001, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(-5.5, 0.5);

//     // pros::delay(2000);


//     while(!Robot::flywheel.reached_target(10)) pros::delay(5); 

//     Robot::INT = -127;
//     pros::delay(300);
//     Robot::INT = 0;

//     Robot::flywheel.set_velocity(2100);

//     pros::delay(750);
    
//     // while(!Robot::flywheel.reached_target(10)) pros::delay(5);

//     Robot::INT = -127;
//     pros::delay(300);
//     Robot::INT = 0;


//     Robot::flywheel.set_velocity(1785);

//     Robot::turn.set_value(1.5, 0.001, 0.0, 6, 0.0);
//      Robot::drive.rotate_to(49.5, 0.5);

//       pros::delay(250);

//      //  Robot::INT = 0;

//       Robot::power.set_value(6.5, 0.01, 0, 2, 10);
//       Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//       Robot::drive.move_to(Pose(24.04, 25.32), 3, 10000, 127, 1000); // 0, -5

//       Robot::INT = 127;

//  //

//      //  Robot::power.set_value(6.5, 0.01, 0, 2, 10);
//      //  Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//      //  Robot::drive.move_to(Pose(38.2, 34.0), 3, 10000, 50); // 0, -5


//      Robot::power.set_value(6.5, 0.01, 0, 2, 10);
//      Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//      Robot::drive.move_to(Pose(46.2, 40.8), 3, 10000, 60); // 0, -5

//       pros::delay(1250);

//       Robot::turn.set_value(1.3, 0.001, 0.0, 6, 0.0);
//       Robot::drive.rotate_to(-38, 0.5);

//      pros::delay(800);

//     // while(!Robot::flywheel.reached_target(10)) pros::delay(5);

//      Robot::INT = -127;
//      pros::delay(365);
//      Robot::INT = 0;
//      pros::delay(300);

//     // while(!Robot::flywheel.reached_target(10)) pros::delay(5);

//     Robot::INT = -127;
//      pros::delay(365);
//      Robot::INT = 0;
//      pros::delay(300);

//     // while(!Robot::flywheel.reached_target(10)) pros::delay(5);

//     Robot::INT = -127;
//      pros::delay(365);
//      Robot::INT = 0;
//      pros::delay(300);
// }

// void awp() {
//     Robot::flywheel.set_velocity(1600);

//     Robot::drive.move(-40, 0);
//     Robot::INT = 100;

//     pros::delay(250);
//     Robot::INT = 127;


//     Robot::power.set_value(12, 0.01, 0, 2, 10);
//     Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
//     Robot::drive.move_to(Pose(0, 4), 1, 10000, 127, 1000); // 0, -5

//     Robot::INT = 127;

//     pros::delay(500);


//     Robot::turn.set_value(1.9, 0.001, 0.0, 6, 0.0);
//     Robot::drive.rotate_to(36, 1);

//     pros::delay(500);

//     Robot::EXP.set_value(true);


// //    //  Robot::INT = 0;
// //
// //    Robot::power.set_value(6.5, 0.01, 0, 2, 10);
// //    Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
// //    Robot::drive.move_to(Pose(24.04, 25.32), 3, 10000, 90, 1000); // 0, -5
// //
// //    Robot::INT = 127;
// //
// //    //
// //    Robot::flywheel.set_velocity(1600);
// //
// //    //  Robot::power.set_value(6.5, 0.01, 0, 2, 10);
// //    //  Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
// //    //  Robot::drive.move_to(Pose(38.2, 34.0), 3, 10000, 50); // 0, -5
// //
// //
// //    Robot::power.set_value(6.5, 0.01, 0, 2, 10);
// //    Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
// //    Robot::drive.move_to(Pose(45.2, 40.8), 3, 10000, 30); // 0, -5
// //
// //    pros::delay(1000);
// //
// //    Robot::turn.set_value(1.3, 0.001, 0.0, 6, 0.0);
// //    Robot::drive.rotate_to(-36.5, 0.5);
// //
// //    pros::delay(1000);
// //
// //    Robot::INT = -127;
// //    pros::delay(400);
// //    Robot::INT = 0;
// //    pros::delay(300);
// //
// //    Robot::INT = -127;
// //    pros::delay(400);
// //    Robot::INT = 0;
// //    pros::delay(600);
// //
// //    Robot::INT = -127;
// //    pros::delay(400);
// //    Robot::INT = 0;
// //    pros::delay(300);
// //
// //    pros::delay(250);
// //
// //    Robot::turn.set_value(1.3, 0.001, 0.0, 6, 0.0);
// //    Robot::drive.rotate_to(42, 0.5);
// //
// //    pros::delay(250);
// //    Robot::INT = 127;
// //
// //    Robot::power.set_value(6.5, 0.01, 0, 2, 10);
// //    Robot::turn.set_value(0, 0, 0.0, 0, 0.0);
// //    Robot::drive.move_to(Pose(96.5, 104.5), 5, 10000, 127, 3000); // 0, -5
// //
// //    pros::delay(500);
// //
// //    Robot::turn.set_value(1.2, 0.001, 0.0, 6, 0.0);
// //    Robot::drive.rotate_to(-90, 1);
// //
// //    pros::delay(250);
// //
// //    Robot::drive.move(-40, 0);
// //    Robot::INT = 100;
// //    pros::delay(520);
// //    Robot::drive.move(40, 0);
// //    Robot::INT = 0;
// //    pros::delay(220);
// //    Robot::drive.move(0, 0);
// }


