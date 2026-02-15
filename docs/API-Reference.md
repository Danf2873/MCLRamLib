# API Reference

Complete reference for all public classes and methods in the MCL + RAMSETE Localization Library.

---

## Table of Contents

- [Pose2D](#pose2d)
- [RobotGeometry](#robotgeometry)
- [DifferentialOdometry](#differentialodometry)
- [MCLLocalizer](#mcllocalizer)
- [FieldMap](#fieldmap)
- [Trajectory](#trajectory)
- [RamseteController](#ramsetecontroller)
- [DistanceSensorConfig](#distancesensorconfig)
- [LocalizationManager](#localizationmanager)
- [ChassisController](#chassiscontroller)

---

## Pose2D

**Header:** `localization/pose2d.hpp`

Represents a 2D pose on the field.

```cpp
struct Pose2D {
    double x;      // X position (inches)
    double y;      // Y position (inches)
    double theta;  // Heading (radians)
};
```

### Static Methods

| Method | Returns | Description |
|---|---|---|
| `normalizeAngle(double angle)` | `double` | Normalizes angle to [-π, π] |
| `distance(Pose2D a, Pose2D b)` | `double` | Euclidean distance between two poses |

---

## RobotGeometry

**Header:** `localization/robot_geometry.hpp`

Robot physical constants.

```cpp
struct RobotGeometry {
    double wheelRadius;
    double trackWidth;
    double gearRatio;
    double encoderTicksPerRev;
    bool hasTrackingWheels;
    double trackWheelRadius;
    double trackWheelTrackWidth;
    double trackWheelCenterOffset;
    Pose2D distanceSensorOffset;
};
```

---

## DifferentialOdometry

**Header:** `localization/odometry.hpp`

Tracks robot pose using motor encoders + IMU.

### Constructor

```cpp
DifferentialOdometry(
    pros::MotorGroup* leftMotors,
    pros::MotorGroup* rightMotors,
    pros::Imu* imu,
    const RobotGeometry& geometry
);
```

### Methods

| Method | Returns | Description |
|---|---|---|
| `update(double dt)` | `void` | Updates pose from encoder deltas |
| `getPose()` | `Pose2D` | Returns current odometry pose |
| `reset(Pose2D pose)` | `void` | Resets pose to a specific value |
| `setTrackingWheels(Rotation* l, Rotation* r, Rotation* m)` | `void` | Configures external tracking wheels |

---

## MCLLocalizer

**Header:** `localization/mcl.hpp`

Monte Carlo Localization particle filter.

### Constructor

```cpp
MCLLocalizer(int numParticles = 200);
```

### Methods

| Method | Returns | Description |
|---|---|---|
| `initialize(minX, maxX, minY, maxY)` | `void` | Scatters particles randomly in a region |
| `predict(v, w, dt)` | `void` | Motion model: applies velocity + noise to particles |
| `updateFromDistanceSensor(dist, map, offset)` | `void` | Measurement model: updates particle weights |
| `resample()` | `void` | Low-variance resampling |
| `estimatePose()` | `Pose2D` | Weighted mean of all particles |
| `getConfidence()` | `double` | Confidence score (0.0 - 1.0) based on particle spread |
| `setPose(pose, stdev)` | `void` | Resets all particles to a pose with noise |

### Noise Parameters

| Parameter | Default | Description |
|---|---|---|
| `alpha1` | `0.05` | Rotation noise from rotation |
| `alpha2` | `0.05` | Rotation noise from translation |
| `alpha3` | `0.05` | Translation noise from translation |
| `alpha4` | `0.05` | Translation noise from rotation |
| `sigmaDistance` | `2.0` | Measurement noise (inches) |

---

## FieldMap

**Header:** `localization/mcl.hpp`

Defines the field walls for MCL ray-casting.

```cpp
struct LineSegment {
    double x1, y1, x2, y2;
};

struct FieldMap {
    std::vector<LineSegment> walls;
    double expectedDistance(const Pose2D& pose) const;
};
```

### `expectedDistance(pose)`

Casts a ray from the given pose along its heading direction and returns the distance to the nearest wall.

---

## Trajectory

**Header:** `localization/trajectory.hpp`

Represents a time-parameterized path.

```cpp
struct TrajectoryPoint {
    double time;   // seconds
    Pose2D pose;   // position and heading
    double v;      // linear velocity (in/s)
    double w;      // angular velocity (rad/s)
};
```

### Constructor

```cpp
Trajectory(const std::vector<TrajectoryPoint>& points);
```

### Methods

| Method | Returns | Description |
|---|---|---|
| `sample(double t)` | `TrajectoryPoint` | Interpolated point at time `t` |
| `totalTime()` | `double` | Duration of the trajectory (seconds) |

---

## RamseteController

**Header:** `localization/ramsete.hpp`

Nonlinear trajectory tracking controller.

### Constructor

```cpp
RamseteController(double b = 2.0, double zeta = 0.7);
```

### Methods

| Method | Returns | Description |
|---|---|---|
| `computeControl(current, desired, v_ref, w_ref)` | `ChassisSpeeds` | Computes corrective velocities |

### ChassisSpeeds

```cpp
struct ChassisSpeeds {
    double v;  // linear velocity
    double w;  // angular velocity
};
```

---

## DistanceSensorConfig

**Header:** `localization/localization_manager.hpp`

Pairs a distance sensor with its mounting configuration.

```cpp
struct DistanceSensorConfig {
    pros::Distance* sensor;     // Pointer to the sensor
    Pose2D offset;              // Mounting offset relative to robot center
    double minRange = 1.0;      // Min valid reading (inches)
    double maxRange = 80.0;     // Max valid reading (inches)
    int minConfidence = 50;     // Min V5 confidence threshold (0-100)
};
```

---

## LocalizationManager

**Header:** `localization/localization_manager.hpp`

High-level façade integrating odometry, MCL, Bayes filter, and sensors.

### Constructor

```cpp
LocalizationManager(
    std::shared_ptr<DifferentialOdometry> odometry,
    std::shared_ptr<MCLLocalizer> mcl,
    const RobotGeometry& geometry
);
```

### Methods

| Method | Returns | Description |
|---|---|---|
| `update(double dt)` | `void` | Updates odometry and MCL prediction |
| `addSensor(config)` | `void` | Registers a distance sensor with offset |
| `updateAllSensors(map)` | `void` | Reads all sensors, updates MCL, resamples |
| `sensorUpdate(dist, map, offset, resample)` | `void` | Manual single-sensor update |
| `resample()` | `void` | Manually triggers MCL resampling |
| `getOdometryPose()` | `Pose2D` | Pure odometry estimate |
| `getFusedPose()` | `Pose2D` | **Bayesian blend** of odometry + MCL |
| `getMCLPose()` | `Pose2D` | Raw particle filter estimate |
| `getMCLConfidence()` | `double` | MCL confidence (0.0 - 1.0) |
| `resetPose(pose)` | `void` | Resets all subsystems to a specific pose |

---

## ChassisController

**Header:** `localization/chassis_controller.hpp`

High-level one-line movement commands.

### Constructor

```cpp
ChassisController(
    pros::MotorGroup* leftMotors,
    pros::MotorGroup* rightMotors,
    LocalizationManager* manager,
    const RobotGeometry& geometry
);
```

### Methods

| Method | Returns | Description |
|---|---|---|
| `moveToPoint(x, y, timeout)` | `void` | Drives to a field coordinate |
| `moveToPose(target, timeout)` | `void` | Drives to a pose (x, y, θ) |
| `follow(trajectory, timeout)` | `void` | Follows a trajectory with RAMSETE |
| `stop()` | `void` | Brakes all motors |

### Default Timeouts

| Method | Default Timeout |
|---|---|
| `moveToPoint` | 5000 ms |
| `moveToPose` | 5000 ms |
| `follow` | 10000 ms |
