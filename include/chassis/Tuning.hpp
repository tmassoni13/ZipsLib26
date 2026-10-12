/*
    Tuning.hpp

    Stores chassis tuning values and turn profiles used by motion commands.
*/

#pragma once

#include <array>
#include "Chassis/ChassisTypes.hpp"

namespace zips {

struct TurnProfileSetting {
  int angleDeg = 0;
  TurnProfile profile;
};

struct Tuning {
  double turnLargeThreshold = 3.0;    // degrees
  double turnSmallThreshold = 1.0;    // degrees
  double turnDtMin = 1e-3;            // seconds
  
  // driveDistance defaults
  double driveMaxVoltage = 12.0;      // volts
  double headingMaxVoltage = 12.0;     // volts
  double driveSettleError = 1.0;        // inches
  double driveSettleTime = 200.0;       // ms
  double driveTimeout = 3000.0;         // ms
  double driveKp = 1.5;
  double driveKi = 0.0;
  double driveKd = 10;
  double driveStartI = 3.0;
  double headingKp = .55;
  double headingKi = 0.0;
  double headingKd = 4.4;
  double headingStartI = 15.0;

  std::array<TurnProfileSetting, 12> turnProfiles;
};

inline Tuning tuning = [] {
  Tuning t;
  t.turnProfiles = {{
    {15,  {3, 0.0, 17}},
    {30,  {3, 0.0, 17}},
    {45,  {3, 0.0, 17}},
    {60,  {3, 0.0, 17}},
    {75,  {3, 0.0, 17}},
    {90,  {3, 0.0, 17}},
    {105, {3, 0.0, 17}},
    {120, {3, 0.0, 17}},
    {135, {3, 0.0, 17}},
    {150, {3, 0.0, 17}},
    {165, {3, 0.0, 17}},
    {180, {3, 0.0, 17}},
  }};
  return t;
}();

}  // namespace zips
