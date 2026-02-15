# Getting Started

This guide walks you through installing MCLRamLib and running your first autonomous routine.

---

## Prerequisites

- **PROS 4.1.1** or later installed ([PROS Installation Guide](https://pros.cs.purdue.edu/v5/getting-started/index.html))
- **C++17** or later (included with PROS 4)
- A VEX V5 robot with:
  - Differential drivetrain (2+ motors per side)
  - V5 Inertial Sensor (IMU)
  - At least one V5 Distance Sensor (more recommended)

---

## Installation

### Step 1: Copy Library Files

Copy the `include/localization/` and `src/localization/` folders into your PROS project:

```
your-project/
├── include/
│   └── localization/
│       ├── pose2d.hpp
│       ├── odometry.hpp
│       ├── mcl.hpp
│       ├── trajectory.hpp
│       ├── ramsete.hpp
│       ├── localization_manager.hpp
│       ├── chassis_controller.hpp
│       ├── robot_geometry.hpp
│       └── robot_ports.hpp
├── src/
│   └── localization/
│       ├── pose2d.cpp
│       ├── odometry.cpp
│       ├── mcl.cpp
│       ├── trajectory.cpp
│       ├── ramsete.cpp
│       ├── localization_manager.cpp
│       ├── chassis_controller.cpp
│       ├── robot_geometry.cpp
│       └── example_auton.cpp
```

### Step 2: Configure Your Robot

Edit `src/localization/robot_geometry.cpp` with your robot's physical measurements:

```cpp
const RobotGeometry DEFAULT_GEOMETRY = {
    2.0,    // wheelRadius (inches) — 4" wheels = 2.0
    12.5,   // trackWidth (inches) — center-to-center of drive wheels
    1.0,    // gearRatio (motor:wheel)
    300.0,  // encoderTicksPerRev (600RPM blue = 300)
    false,  // hasTrackingWheels
    1.375,  // trackWheelRadius (if applicable)
    10.0,   // trackWheelTrackWidth
    0.0,    // trackWheelCenterOffset
    {5.0, 0.0, 0.0} // distanceSensorOffset (default)
};
```

> [!TIP]
> See the [Configuration](Configuration.md) page for detailed explanations of each parameter.

### Step 3: Configure Ports

Edit `src/localization/robot_geometry.cpp` or create a `robot_ports.hpp`:

```cpp
const RobotPorts DEFAULT_PORTS = {
    1,  // leftDrivePort
    2,  // rightDrivePort
    3,  // imuPort
    4,  // distanceSensorPort
    0, 0, 0 // tracking wheel ports (0 = unused)
};
```

### Step 4: Build

```bash
pros make
```

---

## Your First Autonomous

Create a minimal autonomous routine:

```cpp
#include "localization/chassis_controller.hpp"
#include "localization/localization_manager.hpp"

using namespace localization;

void autonomous() {
    // Hardware
    pros::MotorGroup left({1, 2}), right({-3, -4});
    pros::Imu imu(5);
    pros::Distance frontSensor(6);

    // Localization
    auto odom = std::make_shared<DifferentialOdometry>(&left, &right, &imu, DEFAULT_GEOMETRY);
    auto mcl = std::make_shared<MCLLocalizer>(200);
    auto mgr = std::make_shared<LocalizationManager>(odom, mcl, DEFAULT_GEOMETRY);

    // Register sensor
    mgr->addSensor({&frontSensor, {5.0, 0.0, 0.0}});

    // Chassis
    ChassisController chassis(&left, &right, mgr.get(), DEFAULT_GEOMETRY);
    mgr->resetPose({24, 24, 0});

    // Field map (VEX field boundary)
    FieldMap field = {{{0,0,144,0}, {144,0,144,144}, {144,144,0,144}, {0,144,0,0}}};

    // Background update task
    pros::Task updateTask([&]() {
        while (true) {
            mgr->update(0.01);
            mgr->updateAllSensors(field);
            pros::delay(10);
        }
    });

    // Move!
    chassis.moveToPoint(48, 48);
    chassis.moveToPose({72, 72, M_PI/2});
    chassis.stop();
}
```

---

## Next Steps

- [Configuration](Configuration.md) — Deep dive into robot parameters
- [Sensor Setup](Sensor-Setup.md) — Add and configure multiple distance sensors
- [Tuning Guide](Tuning-Guide.md) — Optimize performance for competition
