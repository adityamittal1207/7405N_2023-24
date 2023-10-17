#include "Intake.h"
#include "../Robot.h"

void Intake::lift(){
    pneu_state = 1;
    Robot::INT1.set_value(true);
    Robot::INT2.set_value(true);
}

void Intake::unlift(){
    pneu_state = 0;
    Robot::INT1.set_value(false);
    Robot::INT2.set_value(false);
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