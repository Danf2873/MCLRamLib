# 🤖 MCL + RAMSETE Localization
> **Absolute positioning and path following for PROS 4.1.1**

This library provides MCL-corrected localization for VEX V5 robots. It's designed to be **as easy to use as EZ-Template** — one constructor, simple commands, degrees for angles.

---

## ✨ Features
- 📍 **MCL Fused Pose**: Distance sensors correct odometry drift automatically
- 📡 **Multi-Sensor**: Register unlimited sensors with one line each
- 🏎️ **RAMSETE Path Following**: Smooth, accurate curved paths
- 🧠 **Bayes Filter**: Smart blending of odometry and MCL
- ⚡ **EZ-Template Simple**: One constructor, degrees, no boilerplate

---

## 🚀 Quick Start

### 1. Create Your Chassis (just like EZ-Template!)

```cpp
#include "localization/mcl_chassis.hpp"

// Configure once — that's it!
MCLChassis chassis(
    {1, 2},      // Left motor ports
    {-3, -4},    // Right motor ports (negative = reversed)
    5,           // IMU port
    4.0,         // Wheel diameter (inches)
    12.5         // Track width (inches)
);
```

### 2. Add Sensors (optional but recommended)

```cpp
void initialize() {
    //              port  forward"  left"  facing°
    chassis.add_sensor(6,    5,       0,      0);    // Front
    chassis.add_sensor(7,    0,       5,     90);    // Left
    chassis.add_sensor(8,    0,      -5,    -90);    // Right
}
```

### 3. Write Your Autonomous

```cpp
void autonomous() {
    chassis.set_pose(24, 24, 0);    // Starting position

    chassis.drive_to(48, 48);        // Drive to a point
    chassis.turn_to(90);             // Turn to heading
    chassis.pose_to(72, 72, 180);    // Drive to a pose
    chassis.stop();
}
```

**That's it.** No shared pointers. No update loops. No radians. Just ports, inches, and degrees.

---

## 🛠️ Configuration
Edit your measurements in the constructor:
- **Wheel Diameter**: 4" omnis = `4.0`, 3.25" = `3.25`
- **Track Width**: Center-to-center across your drivetrain
- **Gear Ratio**: Direct drive = `1.0`. 36:60 = `0.6`
- **RPM**: Blue = `600`, Green = `200`, Red = `100`

---

## 📖 Documentation

Full wiki-style documentation is available in the [`docs/`](docs/) folder:

| Page | Description |
|---|---|
| [Getting Started](docs/Getting-Started.md) | Installation & first autonomous |
| [Configuration](docs/Configuration.md) | Robot geometry & port setup |
| [Sensor Setup](docs/Sensor-Setup.md) | Multi-sensor registration & offsets |
| [Movement Commands](docs/Movement-Commands.md) | All chassis commands |
| [Path Following](docs/Path-Following.md) | Trajectories & RAMSETE |
| [Tuning Guide](docs/Tuning-Guide.md) | MCL, RAMSETE, & Bayes filter tuning |
| [API Reference](docs/API-Reference.md) | Complete class & method reference |

---
Developed for the 2024-2025 V5 Competition Season.
