# 🤖 MCL + RAMSETE Localization
> **Absolute positioning and path following for PROS 4.1.1**

This library provides a high-performance localization stack for VEX V5 robots. It fuses **Odometry** (IMU + Encoders) with **Monte Carlo Localization** (Distance Sensors) for sub-inch accuracy across the field.

---

## ✨ Features
- 📍 **MCL Fused Pose**: Monte Carlo Localization corrects wheel slip and drift.
<<<<<<< HEAD
- 📡 **Multi-Sensor Support**: Fuse multiple distance sensors with individual offsets.
- 🏎️ **RAMSETE Control**: Nonlinear trajectory tracking for smooth autonomous.

=======
- 🏎️ **RAMSETE Control**: Nonlinear trajectory tracking for smooth autonomous.
>>>>>>> 089088e80f8fd580dc306f8f312dd20dadc3bf1c
- 🛠️ **Concise API**: Direct commands like `moveToPoint` and `moveToPose`.
- 🧩 **Modular Design**: Easy to integrate, extend, and tune.

---

## 🚀 Quick Start (Concise API)

### 1. Setup Your Chassis
Initialize the `ChassisController` in your autonomous routine:

```cpp
#include "localization/chassis_controller.hpp"

void autonomous() {
    // Standard PROS 4 sensors
    pros::MotorGroup left({1, 2});
    pros::MotorGroup right({-3, -4});
    pros::Imu imu(5);
    pros::Distance dist(6);

    // Initialize Localization & Chassis
    auto odom = std::make_shared<DifferentialOdometry>(&left, &right, &imu, DEFAULT_GEOMETRY);
    auto mcl = std::make_shared<MCLLocalizer>(200);
    auto manager = std::make_shared<LocalizationManager>(odom, mcl, DEFAULT_GEOMETRY);
    
    ChassisController chassis(&left, &right, manager.get(), DEFAULT_GEOMETRY);
    
<<<<<<< HEAD
    // Register sensors with their offsets (once!)
    pros::Distance frontSensor(6);
    pros::Distance leftSensor(7);
    manager->addSensor({&frontSensor, {5.0, 0.0, 0.0}});       // 5" forward, facing front
    manager->addSensor({&leftSensor,  {0.0, 5.0, M_PI/2}});    // 5" left, facing left
    
=======
>>>>>>> 089088e80f8fd580dc306f8f312dd20dadc3bf1c
    manager->resetPose({24.0, 24.0, 0.0});
}
```

<<<<<<< HEAD

### 2. Move with Ease
Sensors update automatically — just call `updateAllSensors` in your loop:

```cpp
while (true) {
    manager->update(0.01);
    manager->updateAllSensors(field);  // All sensors, one call
    
    chassis.moveToPoint(48.0, 48.0);
    chassis.moveToPose({72.0, 72.0, M_PI/2});
}
```

=======
### 2. Move with Ease
Use the short commands to program your autonomous fast:

```cpp
// Move to a field coordinate (inches)
chassis.moveToPoint(48.0, 48.0);

// Move to a specific pose (x, y, theta)
chassis.moveToPose({72.0, 72.0, M_PI/2});

// Follow a pre-planned trajectory path
chassis.follow(autonPath);
```
>>>>>>> 089088e80f8fd580dc306f8f312dd20dadc3bf1c

---

## �️ Configuration
Edit your robot's physical specs in [robot_geometry.cpp](file:///C:/Users/dfoch/OneDrive/Documents/GitHub/MCLRamLib/src/localization/robot_geometry.cpp):
- **Track Width**: Inches between your drive wheels.
- **Wheel Radius**: Radius of your omni/traction wheels.
- **Encoder Ticks**: Resolution of your drivetrain encoders.

---

<<<<<<< HEAD
## 📖 Documentation

Full wiki-style documentation is available in the [`docs/`](docs/) folder:

| Page | Description |
|---|---|
| [Getting Started](docs/Getting-Started.md) | Installation & first autonomous |
| [Configuration](docs/Configuration.md) | Robot geometry & port setup |
| [Sensor Setup](docs/Sensor-Setup.md) | Multi-sensor registration & offsets |
| [Movement Commands](docs/Movement-Commands.md) | ChassisController API |
| [Path Following](docs/Path-Following.md) | Trajectories & RAMSETE |
| [Tuning Guide](docs/Tuning-Guide.md) | MCL, RAMSETE, & Bayes filter tuning |
| [API Reference](docs/API-Reference.md) | Complete class & method reference |

=======
## 📖 Learn More
> [!TIP]
> Use the [Full API Documentation](file:///C:/Users/dfoch/.gemini/antigravity/brain/d4e35621-8b56-4217-b53d-5d00dc832b06/documentation.md) for a deep dive into tuning your MCL particles and RAMSETE gains.

- [Tutorial Walkthrough](file:///C:/Users/dfoch/.gemini/antigravity/brain/d4e35621-8b56-4217-b53d-5d00dc832b06/walkthrough.md)
- [Example Autonomous](file:///C:/Users/dfoch/OneDrive/Documents/GitHub/MCLRamLib/src/localization/example_auton.cpp)
>>>>>>> 089088e80f8fd580dc306f8f312dd20dadc3bf1c

---
Developed for the 2024-2025 V5 Competition Season.

