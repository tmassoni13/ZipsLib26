/*
    intake.cpp

    Intake code for match and autonomous
*/

#include "includes.h"
#include "pros/misc.h"

// Intake motors
pros::MotorGroup intake({-1, 2, 3, 4}, pros::MotorGearset::blue);

// Sets the intake speed (-127 to 127)
void setIntake(int speed) {
  intake.move(speed);
}

// Driver control for intake
void intakeControl() {
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        setIntake(127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        setIntake(-127);
    } else {
        setIntake(0);
    }
}