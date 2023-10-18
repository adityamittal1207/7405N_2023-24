#include "Intake.h"
#include "../Robot.h"

void Intake::expand(){
    pneu_state = 1;
    Robot::INT_EXP.set_value(true);
}

void Intake::retract(){
    pneu_state = 0;
    Robot::INT_EXP.set_value(false);
}


void Intake::intake(){
    Robot::INT.move(-127);
}

void Intake::outtake(){
    Robot::INT.move(127);
}

void Intake::stopintake(){
    Robot::INT.move(0);
}