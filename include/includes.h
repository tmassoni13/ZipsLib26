/*
    includes.h

    Declares shared robot objects, auton functions, and GUI selection helpers.
*/

#pragma once 
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

extern pros::MotorGroup leftDrive;
extern pros::MotorGroup rightDrive;
extern pros::MotorGroup intake;
extern pros::MotorGroup lift;
extern pros::Motor claw;
extern pros::adi::Pneumatics clamp;
extern pros::Rotation clawRotation;
extern pros::Controller master;


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

/**
 * @brief Sets the intake motors to the specified speed.
 *
 * @param speed Motor command from -127 (full reverse) to 127 (full forward) or 0 (off).
 */
void setIntake(int speed);

/**
 * @brief Updates the intake motors based on the controller inputs.
 */
void intakeControl();

/**
 * @brief Configures the lift motors and zeros their integrated encoders.
 *
 * The lift must be physically at height 0 when this function is called.
 */
void liftInit();

/**
 * @brief Selects one of the seven configured lift heights.
 *
 * Values outside the valid range are clamped to heights 0 through 6.
 *
 * @param height Desired lift height index.
 */
void setLiftHeight(int height);

/**
 * @brief Handles the L1 button and updates the lift PID controller.
 *
 * A short L1 press advances one height. Holding L1 for at least 500 ms
 * selects height 0. Call this function repeatedly from opcontrol().
 */
void liftControl();
