# MCL + RAMSETE Localization Library

> Absolute positioning and path following for VEX V5 robots using PROS 4.1.1

---

## What is MCLRamLib?

MCLRamLib is a high-performance localization and trajectory tracking library for VEX V5. It fuses **odometry** (IMU + motor encoders) with **Monte Carlo Localization** (distance sensors) through a **Bayesian filter** to provide sub-inch robot positioning across the entire field.

Unlike pure odometry solutions that drift over time, MCLRamLib continuously corrects its position estimate using distance sensor readings against the known field geometry.

---

## ✨ Key Features

| Feature | Description |
|---|---|
| **MCL Fusion** | Particle filter corrects odometry drift using distance sensors |
| **Bayes Filter** | Confidence-weighted fusion between odometry and MCL estimates |
| **RAMSETE** | Nonlinear trajectory tracking for smooth, accurate paths |
| **Multi-Sensor** | Register unlimited distance sensors with individual offsets |
| **Concise API** | `moveToPoint`, `moveToPose`, `follow` — one-line commands |
| **Modular** | Each component works independently or together |

---

## 📚 Documentation

### Tutorials
1. [Getting Started](Getting-Started.md) — Install and set up the library
2. [Configuration](Configuration.md) — Configure your robot geometry and sensors
3. [Sensor Setup](Sensor-Setup.md) — Register and tune distance sensors
4. [Movement Commands](Movement-Commands.md) — Use the ChassisController API
5. [Path Following](Path-Following.md) — Create and follow trajectories
6. [Tuning Guide](Tuning-Guide.md) — Optimize MCL, RAMSETE, and Bayes filter parameters

### API Reference
7. [API Reference](API-Reference.md) — Complete class and method reference

### Other
8. [About](About.md) — Project goals and credits

---

## Quick Example

```cpp
#include "localization/chassis_controller.hpp"
#include "localization/localization_manager.hpp"

void autonomous() {
    pros::MotorGroup left({1, 2}), right({-3, -4});
    pros::Imu imu(5);
    pros::Distance front(6), side(7);

    auto odom = std::make_shared<DifferentialOdometry>(&left, &right, &imu, DEFAULT_GEOMETRY);
    auto mcl = std::make_shared<MCLLocalizer>(200);
    auto mgr = std::make_shared<LocalizationManager>(odom, mcl, DEFAULT_GEOMETRY);

    mgr->addSensor({&front, {5, 0, 0}});
    mgr->addSensor({&side,  {0, 5, M_PI/2}});

    ChassisController chassis(&left, &right, mgr.get(), DEFAULT_GEOMETRY);
    mgr->resetPose({24, 24, 0});

    // That's it! Move with one-line commands:
    chassis.moveToPoint(48, 48);
    chassis.moveToPose({72, 72, M_PI/2});
}
```
