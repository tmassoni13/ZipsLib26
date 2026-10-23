/*
    claw.cpp

    Claw code for match and autonomous
*/

#include "includes.h"
#include "pros/rotation.hpp"

// Claw motor
pros::Motor claw(25, pros::MotorGearset::green);

// Claw rotation sensor
pros::Rotation clawRotation(25);

// Clamp piston
pros::adi::Pneumatics clamp('A', false);

// Add rest of code once lift code is dialed in
