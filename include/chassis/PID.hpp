/*
    PID.hpp

    Stores the PID class.
*/

#pragma once

namespace zips {

/**
 * @brief PID controller used by robot mechanisms and chassis motion.
 */
class PID {
public:
  PID();

  PID(double kp,
      double ki,
      double kd,
      double startI,
      double settleError,
      int settleTimeMs,
      int timeoutMs,
      int loopTimeMs = 10);

  PID(double kp,
      double ki,
      double kd,
      double startI,
      int loopTimeMs = 10);

  /**
   * @brief Computes the next controller output from the current error.
   */
  double compute(double error);

  /**
   * @brief Clears accumulated and historical controller state.
   */
  void reset();

  /**
   * @brief Reports whether the controller has settled or timed out.
   */
  bool is_settled() const;

private:
  // PID gains
  double kp = 0;
  double ki = 0;
  double kd = 0;
  double starti = 0;
  
  // PID settle/exit values
  double settle_error = 0;
  int settle_time = 0;
  int timeout = 0;
  int loop_time = 10;
  
  // PID runtime values
  double accumulated_error = 0;
  double previous_error = 0;
  int time_spent_settled = 0;
  int time_spent_running = 0;
};

}  // namespace zips
