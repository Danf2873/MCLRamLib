# Getting Started

This guide walks you from zero to a working MCL-localized autonomous in under 5 minutes.

---

## Prerequisites

- **PROS 4.1.1+** installed
- VEX V5 robot with differential drivetrain + IMU
- At least one V5 Distance Sensor (recommended: 2-4)

---

## Installation

Copy `include/localization/` and `src/localization/` into your PROS project. Then `pros make`.

---

## Step 1: Create Your Chassis

In your main file (or a globals file), create the chassis **just like EZ-Template**:

```cpp
#include "localization/mcl_chassis.hpp"
using namespace localization;

MCLChassis chassis(
    {1, 2},      // Left motor ports
    {-3, -4},    // Right motor ports (negative = reversed)
    5,           // IMU port
    4.0,         // Wheel diameter (inches)
    12.5,        // Track width (inches)
    1.0,         // Gear ratio (1.0 = direct drive)
    600          // RPM (600=blue, 200=green, 100=red)
);
```

> [!TIP]
> That single constructor sets up the entire localization stack — odometry, MCL particle filter, Bayes filter, RAMSETE controller, and a background update loop. You don't need to touch any of it.

---

## Step 2: Add Sensors

In `initialize()`, register your distance sensors:

```cpp
void initialize() {
    pros::lcd::initialize();

    // add_sensor(port, forward_offset, left_offset, facing_angle_degrees)
    chassis.add_sensor(6, 5, 0, 0);       // Front: 5" forward, facing 0°
    chassis.add_sensor(7, 0, 5, 90);      // Left: 5" left, facing 90°
    chassis.add_sensor(8, 0, -5, -90);    // Right: 5" right, facing -90°
}
```

Each sensor is registered **once** with where it's mounted. The library reads and fuses them automatically.

---

## Step 3: Write Your Autonomous

```cpp
void autonomous() {
    chassis.set_pose(24, 24, 0);   // Where am I starting?

    chassis.drive_to(48, 48);       // Drive to (48, 48)
    chassis.turn_to(90);            // Turn to face 90°
    chassis.pose_to(72, 72, 180);   // Drive to (72, 72) facing 180°
    chassis.drive_to(24, 24);       // Go home
    chassis.stop();
}
```

**Everything is in inches and degrees.** No radians, no shared pointers, no update loops.

---

## Step 4: Telemetry (Optional)

Print your position to the V5 brain screen:

```cpp
void opcontrol() {
    while (true) {
        pros::lcd::print(0, "X: %.1f  Y: %.1f", chassis.get_x(), chassis.get_y());
        pros::lcd::print(1, "Heading: %.1f°", chassis.get_heading());
        pros::lcd::print(2, "MCL Confidence: %.0f%%", chassis.get_confidence());
        pros::delay(20);
    }
}
```

---

## API Summary

| Command | Description |
|---|---|
| `chassis.set_pose(x, y, heading)` | Set starting position |
| `chassis.drive_to(x, y)` | Drive to a point |
| `chassis.turn_to(heading)` | Turn in place |
| `chassis.pose_to(x, y, heading)` | Drive to a pose |
| `chassis.follow(trajectory)` | Follow a path |
| `chassis.stop()` | Brake |
| `chassis.get_x()` | Current X |
| `chassis.get_y()` | Current Y |
| `chassis.get_heading()` | Current heading (degrees) |
| `chassis.get_confidence()` | MCL confidence (0-100%) |
| `chassis.add_sensor(port, fwd, left, angle)` | Register a sensor |

---

## Next Steps

- [Configuration](Configuration.md) — Understand every parameter
- [Sensor Setup](Sensor-Setup.md) — Add more sensors with custom tuning
- [Tuning Guide](Tuning-Guide.md) — Optimize for competition
