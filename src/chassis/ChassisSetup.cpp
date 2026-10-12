/*
    ChassisSetup.cpp

    Configures the global chassis motors, robot constants, and turn profiles.
*/

#include "Chassis/ChassisController.hpp"
#include "Chassis/Tuning.hpp"

//define motors
pros::MotorGroup leftDrive({-12, -13, 14, -15}, pros::MotorGearset::blue);
pros::MotorGroup rightDrive({19, -18, 17, 16}, pros::MotorGearset::blue);


// Initializes the chassis motors, constants, timeouts, and turn profiles.
void chassisInit() {
  using namespace zips;

  //set drive motor groups
  chassis.setDriveMotors(&leftDrive, &rightDrive);

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
