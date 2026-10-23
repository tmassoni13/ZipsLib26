/*
    ChassisSetup.cpp

    Configures the global chassis motors, robot constants, and turn profiles.
*/

#include "Chassis/ChassisController.hpp"
#include "Chassis/Tuning.hpp"

// Define motors
pros::MotorGroup leftDrive({-7, -8, 9, -10}, pros::MotorGearset::blue);
pros::MotorGroup rightDrive({1, -2, 3, 4}, pros::MotorGearset::blue);

// Define vex inertial sensor if not using OTOS
// pros::Imu chassisImu(11);


// Initializes the chassis motors, constants, timeouts, and turn profiles.
void chassisInit() {
  using namespace zips;

  // Set drive motor groups
  chassis.setDriveMotors(&leftDrive, &rightDrive);

  // Otos is only legal in VEXU, use HeadingSource::Imu for the vex inertial sensor for V5
  chassis.setHeadingSource(HeadingSource::Otos);

  chassis.setRobotConstants({
    .wheelDiameter = 3.25,
    .driveRPM = 480,
    .trackWidth = 10.75,
    .maxVoltage = 12.0
  });

  chassis.setDefaultTimeouts(500, 150);

  for (const auto& entry : tuning.turnProfiles) {
    chassis.setTurnProfile(entry.angleDeg, entry.profile);
  }
}
