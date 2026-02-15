# Sensor Setup

This page covers how to register distance sensors, configure their offsets, and optimize their performance for MCL.

---

## How Sensors Work

Each V5 Distance Sensor measures the distance to the nearest surface in front of it. The MCL system compares these readings against a **field map** (the known wall positions) to determine where the robot most likely is.

**More sensors = more accuracy.** Each sensor provides an independent measurement, and the Bayesian filter fuses them all together.

---

## Registering Sensors

Sensors are registered **once** during initialization using `addSensor()`:

```cpp
pros::Distance frontSensor(6);
pros::Distance leftSensor(7);
pros::Distance rightSensor(8);
pros::Distance backSensor(9);

// Register each with its physical mounting offset
manager->addSensor({&frontSensor, {5.0, 0.0, 0.0}});         // front-facing
manager->addSensor({&leftSensor,  {0.0, 5.0, M_PI / 2}});    // left-facing
manager->addSensor({&rightSensor, {0.0, -5.0, -M_PI / 2}});  // right-facing
manager->addSensor({&backSensor,  {-5.0, 0.0, M_PI}});       // rear-facing
```

---

## Understanding Offsets

The offset `Pose2D{x, y, theta}` describes where the sensor is mounted **relative to the robot center**:

```
          Front (+X)
            ↑
            |
  Left (+Y) ←— [Robot Center] —→ Right (-Y)
            |
            ↓
          Back (-X)
```

| Component | Description |
|---|---|
| `x` | Forward distance from center (positive = forward) |
| `y` | Lateral distance from center (positive = left) |
| `theta` | Direction the sensor faces (0 = forward, π/2 = left, π = backward, -π/2 = right) |

### Example: Sensor mounted 3" forward and 2" to the left, pointing forward

```cpp
Pose2D offset = {3.0, 2.0, 0.0};
```

### Example: Sensor mounted at robot center, pointing 45° left

```cpp
Pose2D offset = {0.0, 0.0, M_PI / 4};
```

---

## DistanceSensorConfig Options

Each sensor has optional tuning parameters:

```cpp
struct DistanceSensorConfig {
    pros::Distance* sensor;     // Pointer to the sensor
    Pose2D offset;              // Mounting offset
    double minRange = 1.0;      // Ignore readings below this (inches)
    double maxRange = 80.0;     // Ignore readings above this (inches)
    int minConfidence = 50;     // Minimum V5 confidence (0-100)
};
```

### Custom Example

```cpp
manager->addSensor({
    &frontSensor,
    {5.0, 0.0, 0.0},  // offset
    2.0,               // minRange: ignore < 2"
    60.0,              // maxRange: ignore > 60"
    70                 // minConfidence: require 70%+ confidence
});
```

> [!TIP]
> In noisy environments (near other robots), increase `minConfidence` to 70-80. On wide-open fields, you can lower it to 30-40 for more frequent updates.

---

## Updating Sensors

In your update loop, call `updateAllSensors()` to read **all** registered sensors in one call:

```cpp
pros::Task updateTask([&]() {
    while (true) {
        manager->update(0.01);            // Odometry + MCL predict
        manager->updateAllSensors(field);  // Read all sensors, update MCL, resample
        pros::delay(10);
    }
});
```

The system will:
1. Read each sensor's distance and confidence
2. Discard readings outside the configured range/confidence
3. Update MCL weights for each valid reading (multiplicative Bayesian update)
4. Resample the particle filter once at the end

---

## Manual Sensor Updates

For advanced use, you can bypass `updateAllSensors` and call `sensorUpdate()` directly:

```cpp
double dist = mySensor.get_distance() / 25.4; // mm → inches
manager->sensorUpdate(dist, field, Pose2D{5, 0, 0}, true);
```

Parameters:
- `dist`: Distance reading in inches
- `field`: The `FieldMap`
- `offset`: `Pose2D` offset (or `std::nullopt` for default)
- `resample`: `true` to resample after this update, `false` to defer

---

## Field Map

The `FieldMap` defines the walls the sensors can "see":

```cpp
FieldMap field = {
    {   // Standard VEX field boundary (12ft × 12ft)
        {0, 0, 144, 0},       // bottom wall
        {144, 0, 144, 144},   // right wall
        {144, 144, 0, 144},   // top wall
        {0, 144, 0, 0}        // left wall
    }
};
```

Each wall is a `LineSegment{x1, y1, x2, y2}` in **inches**.

> [!IMPORTANT]
> Add any field elements that the distance sensor might detect (barriers, goals, etc.) as additional line segments for improved accuracy.

---

## Next Steps

- [Movement Commands](Movement-Commands.md) — Use the ChassisController
- [Tuning Guide](Tuning-Guide.md) — Optimize MCL parameters
