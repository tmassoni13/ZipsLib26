/*
    PID.cpp

    Holds various PID contructors for the PID class.
*/

#include "Chassis/PID.hpp"
#include <cmath>

namespace zips {

PID::PID() = default;

// Full PID constructor
PID::PID(double kp,
         double ki,
         double kd,
         double startI,
         double settleError,
         int settleTimeMs,
         int timeoutMs,
         int loopTimeMs)
    : kp(kp),
      ki(ki),
      kd(kd),
      starti(startI),
      settle_error(settleError),
      settle_time(settleTimeMs),
      timeout(timeoutMs),
      loop_time(loopTimeMs) {}

// headingPID specific constructor
PID::PID(double kp,
         double ki,
         double kd,
         double startI,
         int loopTimeMs)
    : kp(kp),
      ki(ki),
      kd(kd),
      starti(startI),
      loop_time(loopTimeMs) {}


// PID calculations
double PID::compute(double error) {
  if (starti == 0 || std::fabs(error) < starti) {
    accumulated_error += error;
  }
  if ((error > 0 && previous_error < 0) ||
      (error < 0 && previous_error > 0)) {
    accumulated_error = 0;
  }

  const double output =
      kp * error +
      ki * accumulated_error +
      kd * (error - previous_error);

  previous_error = error;

  if (std::fabs(error) < settle_error) {
    time_spent_settled += loop_time;
  } else {
    time_spent_settled = 0;
  }

  time_spent_running += loop_time;
  return output;
}

// Resets PID runtime values
void PID::reset() {
  accumulated_error = 0;
  previous_error = 0;
  time_spent_settled = 0;
  time_spent_running = 0;
}

// Checks if PID is settled and can exit the loop or if PID has timed out
bool PID::is_settled() const {
  if (timeout != 0 && time_spent_running > timeout) return true;
  if (time_spent_settled > settle_time) return true;
  return false;
}

}  // namespace zips
