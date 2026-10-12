/*
    includes.h

    Declares shared robot objects, auton functions, and GUI selection helpers.
*/

#pragma once 
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"

extern pros::MotorGroup leftDrive;
extern pros::MotorGroup rightDrive;
extern pros::MotorGroup intake;
extern pros::Controller master;
extern pros::ADIDigitalOut lift;
extern pros::ADIDigitalOut gate;
extern pros::ADIDigitalOut dropdown;

/**
 * @brief Runs the match autonomous routine.
 */
void match();

/**
 * @brief Runs the skills autonomous routine.
 */
void skills();

/**
 * @brief Initializes the on-brain autonomous selection GUI.
 */
void guiInit();
enum AutonChoice { AutonNone = 0, AutonSkills = 1, AutonMatch = 2 };

/**
 * @brief Gets the autonomous routine selected in the GUI.
 *
 * @return The currently selected autonomous choice.
 */
AutonChoice getSelectedAuton();
