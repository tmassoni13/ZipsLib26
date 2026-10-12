# ZipsLib

ZipsLib is a PROS V5 robot code template for VURC Team ZIPS Akron Robotics. It is built around a reusable chassis controller, OTOS/Pi pose tracking, PID motion helpers, and a simple brain-screen autonomous selector.

## What This Template Can Do

- Configure left and right drive motor groups from one setup file.
- Store robot constants such as wheel diameter, drivetrain RPM, track width, and max voltage.
- Drive with arcade or tank control during opcontrol.
- Run PID-based absolute turns with angle-specific turn profiles.
- Run PID-based drive-distance motions while holding a heading.
- Use default tuning values or override PID, voltage, settle, and timeout settings per motion.
- Run motion callbacks with `onStart`, `onApproach`, and `onExit`.
- Receive pose updates from a Raspberry Pi or coprocessor over the PROS serial stream.
- Convert OTOS sensor pose into robot-center pose.
- Display live X, Y, and theta values on the V5 brain screen.
- Select between match auton and skills auton from the brain screen.

## How To Use The Template

### 1. Set Drive Motors

Open `src/chassis/ChassisSetup.cpp` and update the motor ports:

```cpp
pros::MotorGroup leftDrive({-12, -13, 14, -15}, pros::MotorGearset::blue);
pros::MotorGroup rightDrive({19, -18, 17, 16}, pros::MotorGearset::blue);
```

Use negative ports to reverse motors.

### 2. Set Robot Constants

In `chassisInit()`, update this block:

```cpp
chassis.setRobotConstants({
  .wheelDiameter = 3.25,
  .driveRPM = 480,
  .trackWidth = 10.75,
  .maxVoltage = 12.0
});
```

### 3. Tune Motion Defaults

Open `include/chassis/Tuning.hpp`.

Drive-distance defaults:

```cpp
double driveMaxVoltage = 12.0;
double headingMaxVoltage = 12.0;
double driveSettleError = 1.0;
double driveSettleTime = 200.0;
double driveTimeout = 3000.0;
double driveKp = 1.5;
double driveKi = 0.0;
double driveKd = 10;
```

Turn profiles:

```cpp
{90, {3, 0.0, 17}}
```

That means a 90 degree turn uses `kP = 3`, `kI = 0`, and `kD = 17`.

## How To Use The Chassis

Include the shared headers:

```cpp
#include "main.h"
#include "includes.h"
```

The global chassis object is:

```cpp
zips::chassis
```

### Driver Control

Arcade drive:

```cpp
double forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

zips::chassis.control_arcade(forward, turn, 600);
```

Tank drive:

```cpp
zips::chassis.control_tank(leftPower, rightPower, 600);
```

### Turning

Turn to an absolute heading:

```cpp
zips::chassis.turnTo(90);
```

Turn with a max speed:

```cpp
zips::chassis.turnTo(90, 80);
```

Turn with callbacks or timeout:

```cpp
zips::chassis.turnTo(90, 127, {
  .onStart = [] { /* start mechanism */ },
  .onExit = [] { /* stop mechanism */ },
  .timeout = 1500
});
```

### Driving Distance

Drive forward while holding the current heading:

```cpp
zips::chassis.driveDistance(24);
```

Drive while holding a target heading:

```cpp
zips::chassis.driveDistance(24, 0);
```

Drive with custom voltage limits:

```cpp
zips::chassis.driveDistance(24, 0, 8, 6);
```

Drive with approach triggers:

```cpp
zips::chassis.driveDistance(48, 0, {
  .onStart = [] {
    // runs before motion starts
  },
  .onApproach = {
    {12, [] {
      // runs when the robot is within 12 inches of the target
    }}
  },
  .onExit = [] {
    // runs after motion ends
  },
  .timeout = 3000
});
```

## How To Write Autons

Put autonomous routines in `src/autons.cpp`:

```cpp
void match() {
  zips::chassis.driveDistance(24, 0);
  zips::chassis.turnTo(90);
}

void skills() {
  zips::chassis.driveDistance(48, 0);
}
```

The brain GUI selects between `match()` and `skills()`. `src/main.cpp` calls the selected routine in `autonomous()`.

## How Pose Tracking Works

`main.cpp` starts PiLink during initialization:

```cpp
comm::startPiLink({1, 115200, 64});
comm::requestOtosCalibrate(2500);
```

When a valid pose packet arrives, PiLink calls:

```cpp
zips::chassis.setPoseFromOtos(pose);
```

The GUI reads:

```cpp
zips::chassis.getPose();
```

and displays X, Y, and theta on the V5 brain.

The expected Pi payload format is ASCII:

```text
x,y,theta
```

The packet framing is:

```text
[0] 0xAA
[1] 0x55
[2] payload length
[3..] payload bytes
[last] CRC8
```

## Build And Upload

Build and upload with PROS:

```bash
pros mu
```

If no V5 brain is connected, the project can still compile, but upload will fail with a missing-port message.

## Future Plans

- Point-to-point odometry motions.
- Pure pursuit.
- Multi-point spline pathing.
- More V5 sensor configuration helpers.
- More custom sensor capabilities.
