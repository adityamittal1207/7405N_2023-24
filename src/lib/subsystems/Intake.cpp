#include "Intake.h"
#include "../Robot.h"


void Intake::intake(){
    Robot::INT.move(-127);
}

void Intake::outtake(){
    Robot::INT.move(127);
}

void Intake::stopintake(){
    Robot::INT.move(0);
}