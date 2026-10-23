/*
    lift.cpp

    Lift code for match and autonomous
*/

#include "includes.h"
#include "Chassis/PID.hpp"

#include <algorithm>
#include <array>
#include <cstdint>

// Lift motors
pros::MotorGroup lift({17, -16}, pros::MotorGearset::green, pros::MotorUnits::rotations);

// Lift states using IME rotation values
const std::array<double, 7> liftHeights = {0.0, 50.0, 100.0, 150.0, 200.0, 250.0, 300.0};

// Create liftPID from PID class
zips::PID liftPID(10,
                  0,
                  30,
                  10,
                  5,
                  250,
                  2000,
                  20);

// Initializes button values and sets initial lift height to 0
int liftHeight = 0;
bool l1WasPressed = false;
bool l1WasHeld = false;
std::uint32_t l1PressTime = 0;

// Returns current position of the lift
double getLiftPosition() {
  return (lift.get_position(0) + lift.get_position(1)) / 2.0;
}

// Configure lift motors, zero the encoders, and resets all control states
// Very important that the lift is in the correct down starting position when the code runs
void liftInit() {
  lift.set_brake_mode_all(pros::MotorBrake::hold);
  lift.move_voltage(0);
  lift.tare_position_all();

  liftHeight = 0;
  l1WasPressed = false;
  l1WasHeld = false;
  l1PressTime = 0;
  liftPID.reset();
}

// Change height of the lift using the lift height array
void setLiftHeight(int height) {
  height = std::clamp(height, 0, 6);
  if (height == liftHeight) return;

  liftHeight = height;
  liftPID.reset();
}

// Lift control for driver control 
// L1 press iterates through lift states up to state 6
// L1 hold for >=.5s resets lift to lift state 0 
void liftControl() {
  const bool l1Pressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
  const std::uint32_t now = pros::millis();

  if (l1Pressed && !l1WasPressed) {
    l1PressTime = now;
    l1WasHeld = false;
  } else if (l1Pressed && !l1WasHeld && now - l1PressTime >= 500) {
    setLiftHeight(0);
    l1WasHeld = true;
  } else if (!l1Pressed && l1WasPressed && !l1WasHeld) {
    setLiftHeight(liftHeight + 1);
  }

  l1WasPressed = l1Pressed;

  const double error = liftHeights[liftHeight] - getLiftPosition();
  const int voltage = std::clamp(liftPID.compute(error), -12000.0, 12000.0);

  lift.move_voltage(liftPID.is_settled() ? 0 : voltage);
}
