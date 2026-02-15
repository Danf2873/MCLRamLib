# RAMSETE + MCL Localization Library for PROS 4.1.1

A high-performance, modular localization and trajectory tracking library for VEX V5 robots using the PROS 4 kernel. This library fuses odometry (motor encoders/tracking wheels + IMU) with Monte Carlo Localization (distance sensors) to provide absolute robot positioning on a VEX field.

## Features
- **Differential Odometry**: Supports motor encoders, 2-wheel tracking, and 3-wheel tracking configurations.
- **Monte Carlo Localization (MCL)**: Corrects odometry drift using distance sensors against a field map.
- **RAMSETE Controller**: Advanced nonlinear path following for smooth and accurate autonomous movements.
- **Pose2D System**: Robust 2D vector math for field-relative coordinates.
- **Thread-Safe API**: Designed for easy integration into standard PROS projects.

---

## 🚀 Quick Start Tutorial

### 1. Hardware Setup
Define your robot's physical properties and sensor ports in `src/localization/robot_geometry.cpp`:

```cpp
const RobotGeometry DEFAULT_GEOMETRY = {
    2.0,    // wheelRadius (4" wheels)
    12.5,   // trackWidth (inches)
    1.0,    // gearRatio
    300.0,  // encoderTicksPerRev (600 RPM Blue Cartridge)
    // ... tracking wheel config ...
};
```

### 2. Initialization
In your `autonomous()` or `initialize()` function:

```cpp
#include "localization/localization_manager.hpp"

void autonomous() {
    // 1. Initialize sensor handles
    pros::MotorGroup left({1, 2});
    pros::MotorGroup right({3, 4});
    pros::Imu imu(5);
    pros::Distance dist(6);

    // 2. Create components
    auto odometry = std::make_shared<DifferentialOdometry>(&left, &right, &imu, DEFAULT_GEOMETRY);
    auto mcl = std::make_shared<MCLLocalizer>(200);
    auto manager = std::make_unique<LocalizationManager>(odometry, mcl, DEFAULT_GEOMETRY);

    // 3. Set starting position
    manager->resetPose({24.0, 24.0, 0.0});
}
```

### 3. The Update Loop
Run the localization manager in your main control loop (at least 50-100Hz):

```cpp
while (true) {
    manager->update(0.01); // 10ms delta

    // Sensor-based correction
    double d = dist.get_distance() / 25.4; // mm to inches
    if (dist.get_confidence() > 50) {
        manager->sensorUpdate(d, my_field_map);
    }

    Pose2D pose = manager->getFusedPose();
    printf("Robot Pose: X: %.2f, Y: %.2f, Theta: %.2f\n", pose.x, pose.y, pose.theta);
    
    pros::delay(10);
}
```

### 4. Path Following
Use the `RamseteController` to track a trajectory:

```cpp
RamseteController controller(2.0, 0.7);
TrajectoryPoint target = my_trajectory.sample(currentTime);

ChassisSpeeds output = controller.computeControl(manager->getFusedPose(), target.pose, target.v, target.w);
// Apply output.v and output.w to your drivetrain motors
```

---

## 📁 File Structure
- `include/localization/`: Header files for all modules.
- `src/localization/`: Full source implementations.
- `src/localization/example_auton.cpp`: A complete end-to-end example.

## 📖 Further Documentation
For detailed API references, tuning tips, and advanced usage, please refer to the internal documentation artifacts:
- [Walkthrough & Architecture](file:///C:/Users/dfoch/.gemini/antigravity/brain/d4e35621-8b56-4217-b53d-5d00dc832b06/walkthrough.md)
- [Full API Reference & Tutorial](file:///C:/Users/dfoch/.gemini/antigravity/brain/d4e35621-8b56-4217-b53d-5d00dc832b06/documentation.md)

---
Developed for PROS 4.1.1 V5 Projects.
