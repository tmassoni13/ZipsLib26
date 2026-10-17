/*
    ChassisController.hpp

    Declares the chassis controller interface and global chassis helpers.
*/

#pragma once

#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "ChassisTypes.hpp"

namespace zips {

class ChassisController {
public:
  // --- Setup Functions ---
  /**
   * @brief Sets the left and right drive motor groups used by the chassis.
   *
   * @param left Pointer to the left drive motor group.
   * @param right Pointer to the right drive motor group.
   */
  void setDriveMotors(pros::MotorGroup* left, pros::MotorGroup* right);

  /**
   * @brief Sets the robot constants used by motion calculations.
   *
   * @param constants Wheel, rpm, track width, and voltage constants.
   */
  void setRobotConstants(const RobotConstants& constants);

  /**
   * @brief Selects the sensor used for heading feedback.
   *
   * Pass a configured IMU when selecting HeadingSource::Imu. The IMU is
   * calibrated before it becomes active. Passing nullptr for the IMU safely
   * falls back to OTOS heading.
   *
   * @param source Desired heading source.
   * @param imu IMU to use, or nullptr when selecting OTOS.
   */
  void setHeadingSource(HeadingSource source, pros::Imu* imu = nullptr);

  /**
   * @brief Gets the selected heading source.
   *
   * @return Active heading source.
   */
  HeadingSource getHeadingSource() const;

  /**
   * @brief Stores PID gains for a turn angle profile.
   *
   * @param angleDeg Turn angle in degrees.
   * @param profile PID profile to use for that angle.
   */
  void setTurnProfile(int angleDeg, const TurnProfile& profile);

  // --- Motion Commands ---
  /**
   * @brief Turns the robot to an absolute heading.
   *
   * @param absoluteHeading Target heading in degrees.
   * @param maxSpeed Maximum turn speed on the 0-127 scale.
   * @param options Optional callbacks, timeout, and behavior settings.
   */
  void turnTo(double absoluteHeading, double maxSpeed = 127.0, const MotionOptions& options = {});

  /**
   * @brief Turns the robot to an absolute heading using the default max speed.
   *
   * @param absoluteHeading Target heading in degrees.
   * @param options Optional callbacks, timeout, and behavior settings.
   */
  void turnTo(double absoluteHeading, const MotionOptions& options);

  /**
   * @brief Drives a distance while holding the current heading.
   *
   * @param distance Target distance in inches.
   */
  void driveDistance(float distance);

  /**
   * @brief Drives a distance while holding a target heading.
   *
   * @param distance Target distance in inches.
   * @param heading Heading to hold in degrees.
   */
  void driveDistance(float distance, float heading);

  /**
   * @brief Drives a distance with motion options while holding a target heading.
   *
   * @param distance Target distance in inches.
   * @param heading Heading to hold in degrees.
   * @param options Optional callbacks, timeout, and behavior settings.
   */
  void driveDistance(float distance, float heading, const MotionOptions& options);

  /**
   * @brief Drives a distance with custom drive and heading voltage limits.
   *
   * @param distance Target distance in inches.
   * @param heading Heading to hold in degrees.
   * @param drive_max_voltage Maximum drive voltage in volts.
   * @param heading_max_voltage Maximum heading correction voltage in volts.
   */
  void driveDistance(float distance, float heading, float drive_max_voltage, float heading_max_voltage);

  /**
   * @brief Drives a distance with custom voltage limits and motion options.
   *
   * @param distance Target distance in inches.
   * @param heading Heading to hold in degrees.
   * @param drive_max_voltage Maximum drive voltage in volts.
   * @param heading_max_voltage Maximum heading correction voltage in volts.
   * @param options Optional callbacks, timeout, and behavior settings.
   */
  void driveDistance(float distance, float heading, float drive_max_voltage, float heading_max_voltage,
                     const MotionOptions& options);

  /**
   * @brief Drives a distance with custom voltage limits and timeout.
   *
   * @param distance Target distance in inches.
   * @param heading Heading to hold in degrees.
   * @param drive_max_voltage Maximum drive voltage in volts.
   * @param heading_max_voltage Maximum heading correction voltage in volts.
   * @param drive_timeout Timeout in milliseconds.
   * @param options Optional callbacks and behavior settings.
   */
  void driveDistance(float distance, float heading, float drive_max_voltage, float heading_max_voltage,
                     float drive_timeout, const MotionOptions& options);

  /**
   * @brief Drives a distance with custom voltage limits and settle settings.
   *
   * @param distance Target distance in inches.
   * @param heading Heading to hold in degrees.
   * @param drive_max_voltage Maximum drive voltage in volts.
   * @param heading_max_voltage Maximum heading correction voltage in volts.
   * @param drive_settle_error Distance error considered settled, in inches.
   * @param drive_settle_time Time error must remain settled, in milliseconds.
   * @param drive_timeout Timeout in milliseconds.
   */
  void driveDistance(float distance, float heading, float drive_max_voltage, float heading_max_voltage,
                     float drive_settle_error, float drive_settle_time, float drive_timeout);

  /**
   * @brief Drives a distance with full custom drive and heading PID settings.
   *
   * @param distance Target distance in inches.
   * @param heading Heading to hold in degrees.
   * @param drive_max_voltage Maximum drive voltage in volts.
   * @param heading_max_voltage Maximum heading correction voltage in volts.
   * @param drive_settle_error Distance error considered settled, in inches.
   * @param drive_settle_time Time error must remain settled, in milliseconds.
   * @param drive_timeout Timeout in milliseconds.
   * @param drive_kp Drive proportional gain.
   * @param drive_ki Drive integral gain.
   * @param drive_kd Drive derivative gain.
   * @param drive_starti Error range where drive integral can accumulate.
   * @param heading_kp Heading proportional gain.
   * @param heading_ki Heading integral gain.
   * @param heading_kd Heading derivative gain.
   * @param heading_starti Error range where heading integral can accumulate.
   * @param options Optional callbacks, timeout, and behavior settings.
   */
  void driveDistance(float distance,
                     float heading,
                     float drive_max_voltage,
                     float heading_max_voltage,
                     float drive_settle_error,
                     float drive_settle_time,
                     float drive_timeout,
                     float drive_kp,
                     float drive_ki,
                     float drive_kd,
                     float drive_starti,
                     float heading_kp,
                     float heading_ki,
                     float heading_kd,
                     float heading_starti,
                     const MotionOptions& options = {});

  // --- Motion Control ---
  /**
   * @brief Requests cancellation of the current motion command.
   */
  void cancel(); // cancels current motion

  /**
   * @brief Blocks until the current motion command finishes.
   */
  void wait();   // blocks until current motion completes

  /**
   * @brief Checks whether the chassis is no longer running a motion command.
   *
   * @return True when no motion command is running.
   */
  bool isSettled() const;

  // --- Driver Control ---
  /**
   * @brief Controls the drivetrain with arcade inputs.
   *
   * @param forward Forward command on the -127 to 127 scale.
   * @param turn Turn command on the -127 to 127 scale.
   * @param slewRate Maximum acceleration rate for driver control.
   */
  void control_arcade(double forward, double turn, double slewRate = 600.0);

  /**
   * @brief Controls the drivetrain with tank inputs.
   *
   * @param left Left command on the -127 to 127 scale.
   * @param right Right command on the -127 to 127 scale.
   * @param slewRate Maximum acceleration rate for driver control.
   */
  void control_tank(double left, double right, double slewRate = 600.0);

  // --- Pose Handling ---
  /**
   * @brief Sets the robot pose and resets the OTOS origin.
   *
   * @param newPose Pose to store.
   */
  void setPose(Pose2D newPose);

  /**
   * @brief Applies a robot pose without resetting the OTOS origin.
   *
   * @param newPose Pose to store.
   */
  void applyPose(Pose2D newPose);

  /**
   * @brief Updates the robot pose from an OTOS sensor pose.
   *
   * @param sensorPose Pose reported by the OTOS sensor.
   */
  void setPoseFromOtos(const Pose2D& sensorPose);

  /**
   * @brief Gets the current robot pose.
   *
   * @return Current pose estimate.
   */
  Pose2D getPose() const;

  /**
   * @brief Gets the current heading from the selected heading source.
   *
   * @return Current heading in degrees.
   */
  double getHeading() const;

  /**
   * @brief Gets the timestamp of the most recent OTOS update.
   *
   * @return Last update time in milliseconds, or -1 if none has arrived.
   */
  int getLastOtosUpdateMs() const;

  // --- Debugging ---
  /**
   * @brief Sets default turn settle timeout values.
   *
   * @param largeErrorMs Time allowed at the large error threshold.
   * @param smallErrorMs Time allowed at the small error threshold.
   */
  void setDefaultTimeouts(int largeErrorMs, int smallErrorMs);


private:
  // Internal implementation details will live in .cpp
};

extern ChassisController chassis; // global instance

} // namespace zips

/**
 * @brief Initializes the global chassis instance and tuning settings.
 */
void chassisInit();

// Convert OTOS sensor pose to robot-center pose using tuning offsets.
/**
 * @brief Converts an OTOS sensor pose to a robot-center pose.
 *
 * @param sensorPose Pose reported by the OTOS sensor.
 * @return Pose adjusted to the robot center.
 */
zips::Pose2D otosToRobotPose(const zips::Pose2D& sensorPose);
