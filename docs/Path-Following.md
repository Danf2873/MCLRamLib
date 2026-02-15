# Path Following

This page explains how to create trajectories and follow them using the RAMSETE controller.

---

## What is RAMSETE?

RAMSETE is a **nonlinear feedback controller** designed for differential-drive robots. Unlike simple PID, it handles both linear and angular error simultaneously, producing smooth curved paths.

The controller takes the robot's current pose and a desired trajectory point, then computes the linear velocity `v` and angular velocity `w` needed to stay on path.

---

## Creating a Trajectory

A `Trajectory` is a sequence of `TrajectoryPoint` objects. Each point specifies a timestamp, pose, linear velocity, and angular velocity:

```cpp
struct TrajectoryPoint {
    double time;    // seconds since trajectory start
    Pose2D pose;    // {x, y, theta} at this point
    double v;       // linear velocity (inches/sec)
    double w;       // angular velocity (rad/sec)
};
```

### Example: Straight Line

```cpp
std::vector<TrajectoryPoint> points = {
    {0.0, {24, 24, 0},     20.0, 0.0},   // Start, full speed
    {2.0, {64, 24, 0},     20.0, 0.0},   // Cruise
    {3.0, {84, 24, 0},      0.0, 0.0}    // Stop
};
Trajectory straightLine(points);
```

### Example: Curved Path

```cpp
std::vector<TrajectoryPoint> points = {
    {0.0, {24, 24, 0},          10.0, 0.0},    // Start
    {1.0, {48, 24, 0},          20.0, 0.0},    // Accelerate
    {2.0, {72, 48, M_PI / 2},   10.0, 0.5},    // Curve upward
    {3.0, {72, 72, M_PI / 2},    0.0, 0.0}     // Arrive
};
Trajectory curve(points);
```

> [!TIP]
> The trajectory system uses **linear interpolation** between points, so more points = smoother paths.

---

## Following Trajectories

### Using ChassisController (Recommended)

```cpp
chassis.follow(trajectory);
```

This handles the entire control loop internally.

### Manual RAMSETE Loop (Advanced)

For full control, you can use the `RamseteController` directly:

```cpp
RamseteController controller(2.0, 0.7);  // b, zeta gains

double startTime = pros::millis() / 1000.0;
while (true) {
    double t = pros::millis() / 1000.0 - startTime;
    if (t >= trajectory.totalTime()) break;

    TrajectoryPoint target = trajectory.sample(t);
    Pose2D current = manager->getFusedPose();

    ChassisSpeeds speeds = controller.computeControl(
        current, target.pose, target.v, target.w
    );

    // Convert v, w to left/right wheel speeds
    double leftSpeed = speeds.v - speeds.w * geometry.trackWidth / 2;
    double rightSpeed = speeds.v + speeds.w * geometry.trackWidth / 2;

    left_motors.move_velocity(leftSpeed * ticksPerInch);
    right_motors.move_velocity(rightSpeed * ticksPerInch);

    pros::delay(10);
}
```

---

## Trajectory Sampling

The `Trajectory::sample(t)` method returns the interpolated `TrajectoryPoint` at time `t`:

```cpp
TrajectoryPoint pt = trajectory.sample(1.5);  // At t=1.5 seconds
// pt.pose, pt.v, pt.w are all interpolated
```

If `t` is before the first point, it returns the first point. If `t` is after the last point, it returns the last point.

---

## RAMSETE Parameters

The `RamseteController` has two tuning parameters:

| Parameter | Default | Description |
|---|---|---|
| `b` | `2.0` | Proportional-like gain. Higher = more aggressive convergence |
| `zeta` | `0.7` | Damping ratio. Range: 0 to 1. Higher = more damping |

> [!IMPORTANT]
> Keep `b > 0` and `0 < zeta < 1`. Start with defaults and only adjust if path following is oscillating (`zeta` too low) or too slow to converge (`b` too low).

---

## Next Steps

- [Tuning Guide](Tuning-Guide.md) — Tune RAMSETE, MCL, and Bayes filter parameters
- [API Reference](API-Reference.md) — Full method signatures
