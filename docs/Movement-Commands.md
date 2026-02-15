# Movement Commands

This page covers the `ChassisController` API, which provides short, concise commands for autonomous movement.

---

## Overview

The `ChassisController` wraps the RAMSETE controller, odometry, and motor control into simple one-line commands:

```cpp
ChassisController chassis(&left, &right, manager.get(), DEFAULT_GEOMETRY);

chassis.moveToPoint(48, 48);              // Move to a point
chassis.moveToPose({72, 72, M_PI/2});     // Move to a pose (x, y, heading)
chassis.follow(trajectory);               // Follow a trajectory
chassis.stop();                           // Brake
```

---

## `moveToPoint(x, y, timeout)`

Drives the robot to a target **(x, y)** coordinate on the field. The robot will face toward the target point automatically.

```cpp
chassis.moveToPoint(48.0, 48.0);           // Default 5s timeout
chassis.moveToPoint(48.0, 48.0, 3000);     // 3 second timeout
```

| Parameter | Type | Default | Description |
|---|---|---|---|
| `x` | `double` | — | Target X position (inches) |
| `y` | `double` | — | Target Y position (inches) |
| `timeout` | `int` | `5000` | Maximum time in milliseconds |

---

## `moveToPose(target, timeout)`

Drives the robot to a target pose **(x, y, θ)**. The robot will arrive at the target position **and** heading.

```cpp
chassis.moveToPose({72.0, 72.0, M_PI / 2});         // Default timeout
chassis.moveToPose({72.0, 72.0, M_PI / 2}, 4000);   // 4 second timeout
```

| Parameter | Type | Default | Description |
|---|---|---|---|
| `target` | `Pose2D` | — | Target pose {x, y, theta} |
| `timeout` | `int` | `5000` | Maximum time in milliseconds |

> [!NOTE]
> `theta` is in **radians**. Use `M_PI` for 180°, `M_PI/2` for 90°, etc.

---

## `follow(trajectory, timeout)`

Follows a pre-planned `Trajectory` using the RAMSETE controller for smooth, accurate path tracking.

```cpp
std::vector<TrajectoryPoint> points = {
    {0.0, {24, 24, 0}, 10.0, 0.0},
    {1.0, {48, 24, 0}, 20.0, 0.0},
    {2.0, {72, 48, M_PI/2}, 10.0, 0.0},
    {3.0, {72, 72, M_PI/2}, 0.0, 0.0}
};
Trajectory traj(points);

chassis.follow(traj);           // Default timeout
chassis.follow(traj, 10000);    // 10 second timeout
```

| Parameter | Type | Default | Description |
|---|---|---|---|
| `trajectory` | `Trajectory` | — | The path to follow |
| `timeout` | `int` | `10000` | Maximum time in milliseconds |

---

## `stop()`

Immediately brakes all motors.

```cpp
chassis.stop();
```

---

## Full Autonomous Example

```cpp
void autonomous() {
    // ... initialization ...

    // Start at bottom-left corner
    manager->resetPose({24, 24, 0});

    // Drive across the field
    chassis.moveToPoint(48, 48);

    // Turn and drive to a specific pose
    chassis.moveToPose({72, 72, M_PI / 2});

    // Follow a curved path back
    Trajectory returnPath(/* ... */);
    chassis.follow(returnPath);

    // Stop at home
    chassis.moveToPoint(24, 24);
    chassis.stop();
}
```

---

## Next Steps

- [Path Following](Path-Following.md) — Creating trajectories in detail
- [Tuning Guide](Tuning-Guide.md) — Adjust RAMSETE gains
