/*
    ChassisController.cpp

    Implements chassis pose handling, driver control, turning, and distance
    movement.
*/

#include "Chassis/ChassisController.hpp"
#include "Chassis/PID.hpp"
#include "Chassis/Tuning.hpp"
#include <algorithm>
#include <cstdint>
#include <map>
#include <cmath>

namespace zips {

// ===================== GLOBAL STATE =====================
static bool isRunning = false;
static bool wasCancelled = false;

static int globalLargeErrorTimeoutMs = 500;
static int globalSmallErrorTimeoutMs = 150;

static int lastOtosUpdateMs = -1;
static bool otosOriginSet = false;
static Pose2D otosOrigin = {0, 0, 0};

// ========== GLOBAL CONTROLLER INSTANCE ==========
ChassisController chassis;

// ========== PRIVATE MEMBERS ==========
static pros::MotorGroup* leftMotors = nullptr;
static pros::MotorGroup* rightMotors = nullptr;
static pros::Imu* headingImu = nullptr;

static Pose2D pose = {0, 0, 0};
static RobotConstants constants;
static HeadingSource headingSource = HeadingSource::Otos;
static double imuHeadingOffset = 0.0;

// Local OTOS mount offsets
static double otosOffsetForward = 0.0; // inches
static double otosOffsetLeft = 0.0;    // inches

// Applies acceleration limiting to a driver command.
static double applySlew(double target, double current, double dt, double rate) {
  if (rate <= 0.0 || dt <= 0.0) return target;

  if (std::abs(target) <= std::abs(current) || (target == 0.0) ||
      (target > 0.0 && current < 0.0) || (target < 0.0 && current > 0.0)) {
    return target;
  }

  const double maxDelta = rate * dt;
  const double delta = target - current;
  if (delta > maxDelta) return current + maxDelta;
  if (delta < -maxDelta) return current - maxDelta;
  return target;
}

// Applies a basic slew rate limit to a target value.
static double applySlewRate(double target, double current, double rate, double dt) {
  if (rate <= 0.0) return target;
  const double maxChange = rate * dt;
  const double change = target - current;
  if (std::abs(change) > std::abs(maxChange)) {
    return current + (maxChange * ((change > 0.0) ? 1.0 : -1.0));
  }
  return target;
}

// Wraps an angle in radians to the -pi to pi range.
static double wrapRadians(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

// Computes wrapped angular error in radians.
static double angleErrorRad(double target, double position) {
  return wrapRadians(target - position);
}

// Converts wheel rotation degrees to linear inches.
static double degreesToInches(double deg, double wheelDiameter) {
  return (deg / 360.0) * (wheelDiameter * M_PI);
}

// Wraps an angle in degrees to the -180 to 180 range.
static double wrapDegrees(double angleDeg) {
  return std::remainder(angleDeg, 360.0);
}

// Reads heading from the selected sensor using the OTOS CCW-positive convention.
static double readHeading() {
  if (headingSource == HeadingSource::Imu && headingImu != nullptr &&
      !headingImu->is_calibrating()) {
    const double rotation = headingImu->get_rotation();
    if (std::isfinite(rotation)) {
      // V5 IMU rotation is clockwise-positive; chassis heading is CCW-positive.
      return wrapDegrees(imuHeadingOffset - rotation);
    }
  }

  return pose.theta;
}

// ===================== OTOS -> ROBOT CENTER POSE =====================
// sensorPose is in project frame:
//   +Y forward, +X right, theta=0 along +Y, CCW+
// Converts an OTOS sensor pose into the robot-center pose.
Pose2D otosToRobotPose(const Pose2D& sensorPose) {
  const double h = sensorPose.theta * (M_PI / 180.0);

  const double forward = otosOffsetForward;
  const double left = otosOffsetLeft;

  // forward dir = (sin h, cos h)
  // left dir    = (-cos h, sin h)
  const double off_x = forward * std::sin(h) + left * (-std::cos(h));
  const double off_y = forward * std::cos(h) + left * ( std::sin(h));

  Pose2D out = sensorPose;
  out.x = sensorPose.x - off_x;
  out.y = sensorPose.y - off_y;
  return out;
}

// ===================== SETUP =====================
// Stores the motor groups used for chassis movement.
void ChassisController::setDriveMotors(pros::MotorGroup* left, pros::MotorGroup* right) {
  leftMotors = left;
  rightMotors = right;
}

// Stores the robot constants used by drive calculations.
void ChassisController::setRobotConstants(const RobotConstants& rc) {
  constants = rc;
}

// Selects OTOS or a V5 IMU as the heading feedback source.
void ChassisController::setHeadingSource(HeadingSource source, pros::Imu* imu) {
  if (source == HeadingSource::Imu && imu == nullptr) {
    headingSource = HeadingSource::Otos;
    headingImu = nullptr;
    return;
  }

  headingSource = source;
  headingImu = (source == HeadingSource::Imu) ? imu : nullptr;

  if (headingImu != nullptr) {
    headingImu->reset(true);
    const double rotation = headingImu->get_rotation();
    imuHeadingOffset = pose.theta + (std::isfinite(rotation) ? rotation : 0.0);
  }
}

// Returns the active heading feedback source.
HeadingSource ChassisController::getHeadingSource() const {
  return headingSource;
}

// Sets the default timeout windows for turn settling.
void ChassisController::setDefaultTimeouts(int largeErrorMs, int smallErrorMs) {
  globalLargeErrorTimeoutMs = largeErrorMs;
  globalSmallErrorTimeoutMs = smallErrorMs;
}

// Returns the current robot pose estimate.
Pose2D ChassisController::getPose() const {
  Pose2D currentPose = pose;
  currentPose.theta = readHeading();
  return currentPose;
}

// Returns the current robot heading.
double ChassisController::getHeading() const {
  return readHeading();
}

// Returns the timestamp for the most recent OTOS update.
int ChassisController::getLastOtosUpdateMs() const {
  return lastOtosUpdateMs;
}

// Sets the robot pose and resets the OTOS origin.
void ChassisController::setPose(Pose2D newPose) {
  pose = newPose;
  otosOriginSet = false;

  if (headingSource == HeadingSource::Imu && headingImu != nullptr) {
    const double rotation = headingImu->get_rotation();
    if (std::isfinite(rotation)) {
      imuHeadingOffset = newPose.theta + rotation;
    }
  }
}

// Applies a robot pose without resetting the OTOS origin.
void ChassisController::applyPose(Pose2D newPose) {
  pose = newPose;
}

// Converts and stores a new OTOS pose update.
void ChassisController::setPoseFromOtos(const Pose2D& sensorPose) {
  Pose2D robotPose = otosToRobotPose(sensorPose);

  if (!otosOriginSet) {
    otosOrigin = robotPose;
    otosOriginSet = true;
  }

  Pose2D localPose = robotPose;
  localPose.x -= otosOrigin.x;
  localPose.y -= otosOrigin.y;
  localPose.theta = std::remainder(robotPose.theta - otosOrigin.theta, 360.0);

  applyPose(localPose);
  lastOtosUpdateMs = pros::millis();
}

// ===================== TURN CONTROL =====================
static std::map<int, TurnProfile> turnProfiles;
static int lastDriveTimeMs = -1;
static double lastLeftCmd = 0.0;
static double lastRightCmd = 0.0;

// Stores a turn PID profile for a given angle.
void ChassisController::setTurnProfile(int angleDeg, const TurnProfile& profile) {
  turnProfiles[angleDeg] = profile;
}

// Turns the robot to an absolute heading using the selected turn profile.
void ChassisController::turnTo(double targetHeading, double maxSpeed, const MotionOptions& options) {
  if (!leftMotors || !rightMotors) return;

  isRunning = true;
  wasCancelled = false;

  const double startHeading = readHeading();
  const double headingError = std::remainder(targetHeading - startHeading, 360.0);
  const double absAngle = std::abs(headingError);

  int closest = 15 * std::round(absAngle / 15.0);
  if (turnProfiles.find(closest) == turnProfiles.end()) {
    leftMotors->move_voltage(0);
    rightMotors->move_voltage(0);
    isRunning = false;
    return;
  }
  TurnProfile profile = turnProfiles[closest];

  double maxVel = std::clamp(maxSpeed, 1.0, 127.0);
  double integral = 0.0;
  double lastError = 0.0;

  std::uint32_t startTime = pros::millis();
  std::uint32_t t0 = startTime;
  const double timeoutMs = (options.timeout > 0.0) ? options.timeout : 1000.0;

  int largeErrorStart = -1;
  int smallErrorStart = -1;

  if (options.onStart) options.onStart();

  while (true) {
    if (wasCancelled) break;

    std::uint32_t now = pros::millis();
    double dt = static_cast<double>(now - t0) / 1000.0;
    if (dt < tuning.turnDtMin) dt = tuning.turnDtMin;
    t0 = now;

    double current = readHeading();
    double error = std::remainder(targetHeading - current, 360.0);
    double absErr = std::abs(error);

    if (absErr <= tuning.turnLargeThreshold) {
      if (largeErrorStart == -1) largeErrorStart = static_cast<int>(now);
    } else {
      largeErrorStart = -1;
    }

    if (absErr <= tuning.turnSmallThreshold) {
      if (smallErrorStart == -1) smallErrorStart = static_cast<int>(now);
    } else {
      smallErrorStart = -1;
    }

    bool largeDone = largeErrorStart > 0 &&
      static_cast<int>(now - static_cast<std::uint32_t>(largeErrorStart)) >= globalLargeErrorTimeoutMs;
    bool smallDone = smallErrorStart > 0 &&
      static_cast<int>(now - static_cast<std::uint32_t>(smallErrorStart)) >= globalSmallErrorTimeoutMs;
    bool userTimeout = static_cast<double>(now - startTime) >= timeoutMs;

    if (largeDone || smallDone || userTimeout) break;

    const double loop_scale = (dt > 1e-6) ? (dt / 0.01) : 1.0; // normalize to 10ms loop
    integral += error * loop_scale;

    const double maxIntegral = maxVel;
    if (integral > maxIntegral) integral = maxIntegral;
    if (integral < -maxIntegral) integral = -maxIntegral;

    double derivative = (error - lastError) / loop_scale;
    double output = (profile.kP * error) + (profile.kI * integral) + (profile.kD * derivative);
    lastError = error;

    double voltage = output * 12000.0 / maxVel;
    voltage = std::clamp(voltage, -12000.0, 12000.0);

    leftMotors->move_voltage(-voltage);
    rightMotors->move_voltage(voltage);

    pros::delay(10);
  }

  leftMotors->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rightMotors->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  leftMotors->move_voltage(0);
  rightMotors->move_voltage(0);

  if (options.onExit) options.onExit();
  isRunning = false;
}

// Turns the robot to an absolute heading using the default max speed.
void ChassisController::turnTo(double targetHeading, const MotionOptions& options) {
  turnTo(targetHeading, 127.0, options);
}

// Drives a distance while holding the current heading.
void ChassisController::driveDistance(float distance) {
  driveDistance(distance,
                static_cast<float>(readHeading()),
                static_cast<float>(tuning.driveMaxVoltage),
                static_cast<float>(tuning.headingMaxVoltage),
                static_cast<float>(tuning.driveSettleError),
                static_cast<float>(tuning.driveSettleTime),
                static_cast<float>(tuning.driveTimeout),
                static_cast<float>(tuning.driveKp),
                static_cast<float>(tuning.driveKi),
                static_cast<float>(tuning.driveKd),
                static_cast<float>(tuning.driveStartI),
                static_cast<float>(tuning.headingKp),
                static_cast<float>(tuning.headingKi),
                static_cast<float>(tuning.headingKd),
                static_cast<float>(tuning.headingStartI));
}

// Drives a distance while holding the requested heading.
void ChassisController::driveDistance(float distance, float heading) {
  driveDistance(distance,
                heading,
                static_cast<float>(tuning.driveMaxVoltage),
                static_cast<float>(tuning.headingMaxVoltage),
                static_cast<float>(tuning.driveSettleError),
                static_cast<float>(tuning.driveSettleTime),
                static_cast<float>(tuning.driveTimeout),
                static_cast<float>(tuning.driveKp),
                static_cast<float>(tuning.driveKi),
                static_cast<float>(tuning.driveKd),
                static_cast<float>(tuning.driveStartI),
                static_cast<float>(tuning.headingKp),
                static_cast<float>(tuning.headingKi),
                static_cast<float>(tuning.headingKd),
                static_cast<float>(tuning.headingStartI),
                {});
}

// Drives a distance with motion options while holding a heading.
void ChassisController::driveDistance(float distance,
                                      float heading,
                                      const MotionOptions& options) {
  driveDistance(distance,
                heading,
                static_cast<float>(tuning.driveMaxVoltage),
                static_cast<float>(tuning.headingMaxVoltage),
                static_cast<float>(tuning.driveSettleError),
                static_cast<float>(tuning.driveSettleTime),
                static_cast<float>(tuning.driveTimeout),
                static_cast<float>(tuning.driveKp),
                static_cast<float>(tuning.driveKi),
                static_cast<float>(tuning.driveKd),
                static_cast<float>(tuning.driveStartI),
                static_cast<float>(tuning.headingKp),
                static_cast<float>(tuning.headingKi),
                static_cast<float>(tuning.headingKd),
                static_cast<float>(tuning.headingStartI),
                options);
}

// Drives a distance with custom drive and heading voltage limits.
void ChassisController::driveDistance(float distance,
                                      float heading,
                                      float drive_max_voltage,
                                      float heading_max_voltage) {
  driveDistance(distance,
                heading,
                drive_max_voltage,
                heading_max_voltage,
                static_cast<float>(tuning.driveSettleError),
                static_cast<float>(tuning.driveSettleTime),
                static_cast<float>(tuning.driveTimeout),
                static_cast<float>(tuning.driveKp),
                static_cast<float>(tuning.driveKi),
                static_cast<float>(tuning.driveKd),
                static_cast<float>(tuning.driveStartI),
                static_cast<float>(tuning.headingKp),
                static_cast<float>(tuning.headingKi),
                static_cast<float>(tuning.headingKd),
                static_cast<float>(tuning.headingStartI),
                {});
}

// Drives a distance with custom voltage limits and motion options.
void ChassisController::driveDistance(float distance,
                                      float heading,
                                      float drive_max_voltage,
                                      float heading_max_voltage,
                                      const MotionOptions& options) {
  driveDistance(distance,
                heading,
                drive_max_voltage,
                heading_max_voltage,
                static_cast<float>(tuning.driveSettleError),
                static_cast<float>(tuning.driveSettleTime),
                static_cast<float>(tuning.driveTimeout),
                static_cast<float>(tuning.driveKp),
                static_cast<float>(tuning.driveKi),
                static_cast<float>(tuning.driveKd),
                static_cast<float>(tuning.driveStartI),
                static_cast<float>(tuning.headingKp),
                static_cast<float>(tuning.headingKi),
                static_cast<float>(tuning.headingKd),
                static_cast<float>(tuning.headingStartI),
                options);
}

// Drives a distance with custom voltage limits and timeout.
void ChassisController::driveDistance(float distance,
                                      float heading,
                                      float drive_max_voltage,
                                      float heading_max_voltage,
                                      float drive_timeout,
                                      const MotionOptions& options) {
  driveDistance(distance,
                heading,
                drive_max_voltage,
                heading_max_voltage,
                static_cast<float>(tuning.driveSettleError),
                static_cast<float>(tuning.driveSettleTime),
                drive_timeout,
                static_cast<float>(tuning.driveKp),
                static_cast<float>(tuning.driveKi),
                static_cast<float>(tuning.driveKd),
                static_cast<float>(tuning.driveStartI),
                static_cast<float>(tuning.headingKp),
                static_cast<float>(tuning.headingKi),
                static_cast<float>(tuning.headingKd),
                static_cast<float>(tuning.headingStartI),
                options);
}

// Drives a distance with custom voltage limits and settle settings.
void ChassisController::driveDistance(float distance,
                                      float heading,
                                      float drive_max_voltage,
                                      float heading_max_voltage,
                                      float drive_settle_error,
                                      float drive_settle_time,
                                      float drive_timeout) {
  driveDistance(distance,
                heading,
                drive_max_voltage,
                heading_max_voltage,
                drive_settle_error,
                drive_settle_time,
                drive_timeout,
                static_cast<float>(tuning.driveKp),
                static_cast<float>(tuning.driveKi),
                static_cast<float>(tuning.driveKd),
                static_cast<float>(tuning.driveStartI),
                static_cast<float>(tuning.headingKp),
                static_cast<float>(tuning.headingKi),
                static_cast<float>(tuning.headingKd),
                static_cast<float>(tuning.headingStartI),
                {});
}

// Drives a distance using full custom drive and heading PID settings.
void ChassisController::driveDistance(float distance,
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
                                      const MotionOptions& options) {
  if (!leftMotors || !rightMotors) return;

  isRunning = true;
  wasCancelled = false;

  if (options.onStart) options.onStart();

  PID drivePID(drive_kp, drive_ki, drive_kd, drive_starti,
               drive_settle_error,
               (int)drive_settle_time,
               (int)drive_timeout);

  auto get_absolute_heading = [&]() -> double {
    return readHeading();
  };

  auto reduce_negative_180_to_180 = [&](double deg) -> double {
    while (deg > 180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
  };

  PID headingPID(heading_kp, heading_ki, heading_kd, heading_starti);

  const double start_avg_in =
    (degreesToInches(leftMotors->get_position(),  constants.wheelDiameter) +
     degreesToInches(rightMotors->get_position(), constants.wheelDiameter)) / 2.0;

  double avg_in = start_avg_in;
  std::vector<DistanceTrigger> triggers = options.onApproach;

  while (!drivePID.is_settled()) {
    if (wasCancelled) break;

    avg_in =
      (degreesToInches(leftMotors->get_position(),  constants.wheelDiameter) +
       degreesToInches(rightMotors->get_position(), constants.wheelDiameter)) / 2.0;

    const double drive_error =
      (double)distance + start_avg_in - avg_in;

    const double heading_error =
      reduce_negative_180_to_180(get_absolute_heading() - heading);

    const double remaining = std::abs(drive_error);
    for (auto& trig : triggers) {
      if (!trig.triggered && remaining <= trig.distanceBeforeTarget) {
        if (trig.action) trig.action();
        trig.triggered = true;
      }
    }

    double drive_output   = drivePID.compute(drive_error);
    double heading_output = headingPID.compute(heading_error);

    drive_output =
      std::clamp(drive_output,
                 -(double)drive_max_voltage,
                  (double)drive_max_voltage);

    heading_output =
      std::clamp(heading_output,
                 -(double)heading_max_voltage,
                  (double)heading_max_voltage);

    const double left_volts  = drive_output + heading_output;
    const double right_volts = drive_output - heading_output;

    leftMotors->move_voltage(
      (int)std::clamp(left_volts * 1000.0, -12000.0, 12000.0));
    rightMotors->move_voltage(
      (int)std::clamp(right_volts * 1000.0, -12000.0, 12000.0));

    pros::delay(10);
  }

  leftMotors->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rightMotors->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  leftMotors->move_voltage(0);
  rightMotors->move_voltage(0);

  if (options.onExit) options.onExit();
  isRunning = false;
}

// ===================== DRIVER CONTROL =====================
// Converts arcade drive commands into tank drive commands.
void ChassisController::control_arcade(double forward, double turn, double slewRate) {
  const double left = forward + turn;
  const double right = forward - turn;
  control_tank(left, right, slewRate);
}

// Sends tank drive commands to the drivetrain with slew limiting.
void ChassisController::control_tank(double left, double right, double slewRate) {
  if (!leftMotors || !rightMotors) return;

  const int now = pros::millis();
  double dt = 0.0;
  if (lastDriveTimeMs >= 0) dt = (now - lastDriveTimeMs) / 1000.0;
  lastDriveTimeMs = now;

  double targetLeft = std::clamp(left, -127.0, 127.0);
  double targetRight = std::clamp(right, -127.0, 127.0);

  targetLeft = applySlew(targetLeft, lastLeftCmd, dt, slewRate);
  targetRight = applySlew(targetRight, lastRightCmd, dt, slewRate);

  lastLeftCmd = targetLeft;
  lastRightCmd = targetRight;

  leftMotors->move(targetLeft);
  rightMotors->move(targetRight);
}

// ===================== MOTION CONTROL =====================
// Requests cancellation of the active motion command.
void ChassisController::cancel() {
  wasCancelled = true;
}

// Blocks until the active motion command finishes.
void ChassisController::wait() {
  while (isRunning) pros::delay(5);
}

// Returns whether no motion command is running.
bool ChassisController::isSettled() const {
  return !isRunning;
}

} // namespace zips
