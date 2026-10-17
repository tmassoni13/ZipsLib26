/*
    ChassisTypes.hpp

    Defines chassis pose, motion, trigger, and option data structures.
*/

#pragma once

#include <functional>
#include <vector>
#include <cmath>

namespace zips {

  enum class HeadingSource {
    Otos,
    Imu
  };

  // 2D pose struct with position and heading
  struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0; // degrees
  };

  // Turn profile struct used by the turn controller
  struct TurnProfile {
    double kP = 1.0;
    double kI = 0.0;
    double kD = 0.0;
  };

  // Robot motion constants for scaling
  struct RobotConstants {
    double wheelDiameter = 4.125; // inches
    double driveRPM = 200;        // measured RPM of drive
    double trackWidth = 11.5;     // inches
    double maxVoltage = 12.0;     // volts
  };

  // Structure to define trigger actions based on distance before target
  struct DistanceTrigger {
    double distanceBeforeTarget = 0.0;
    std::function<void()> action = nullptr;
    bool triggered = false; // internally managed
  };

  // User-facing options to configure any motion command
  struct MotionOptions {
    std::function<void()> onStart = nullptr;
    std::vector<DistanceTrigger> onApproach;
    std::function<void()> onExit = nullptr;

    double timeout = 0.0;    // Optional per-function override (ms); 0 = use defaults
    bool reversed = false;
  };
} // namespace zips
