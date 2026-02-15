# Configuration

This page explains every parameter in `RobotGeometry` and `RobotPorts` and how to measure them accurately.

---

## RobotGeometry

All measurements are in **inches**. Accuracy here directly impacts localization quality.

### Drivetrain Parameters

| Parameter | Description | How to Measure |
|---|---|---|
| `wheelRadius` | Radius of your drive wheels | Diameter ÷ 2. Standard 4" omnis = `2.0` |
| `trackWidth` | Distance between left and right wheel centers | Measure center-to-center across the drivetrain |
| `gearRatio` | Motor rotations per wheel rotation | If direct drive: `1.0`. If 36:60: `0.6` |
| `encoderTicksPerRev` | Motor encoder ticks per revolution | Blue 600RPM: `300`, Green 200RPM: `900`, Red 100RPM: `1800` |

> [!IMPORTANT]
> `trackWidth` is the most critical measurement. Even 0.5" of error causes significant heading drift. Measure carefully and tune empirically.

### Tracking Wheel Parameters (Optional)

| Parameter | Description |
|---|---|
| `hasTrackingWheels` | Set to `true` if using external tracking wheels |
| `trackWheelRadius` | Radius of your tracking wheels |
| `trackWheelTrackWidth` | Distance between left and right tracking wheels |
| `trackWheelCenterOffset` | Forward/backward offset of the center tracking wheel from robot center. Positive = forward |

### Distance Sensor Offset (Default)

```cpp
Pose2D distanceSensorOffset; // {x_forward, y_left, facing_angle}
```

- `x`: Forward distance from robot center (positive = forward)
- `y`: Lateral distance from robot center (positive = left)
- `theta`: Angle the sensor faces relative to robot heading (0 = forward, π/2 = left)

> [!NOTE]
> This is the **default** offset used when calling `sensorUpdate()` without an explicit offset. If you use `addSensor()`, each sensor gets its own offset and this default is only a fallback.

---

## RobotPorts

```cpp
struct RobotPorts {
    int8_t leftDrivePort;
    int8_t rightDrivePort;
    int8_t imuPort;
    int8_t distanceSensorPort;
    int8_t leftTrackPort;    // 0 = unused
    int8_t rightTrackPort;   // 0 = unused
    int8_t middleTrackPort;  // 0 = unused
};
```

> [!TIP]
> Use **negative port numbers** for reversed motors. For example, if your right motors should spin in reverse, use `-3` instead of `3`.

---

## Motor Groups

If you have more than one motor per side, use `pros::MotorGroup`:

```cpp
// 2-motor drive (ports 1,2 left | -3,-4 right)
pros::MotorGroup left({1, 2});
pros::MotorGroup right({-3, -4});

// 3-motor drive
pros::MotorGroup left({1, 2, 3});
pros::MotorGroup right({-4, -5, -6});
```

---

## Empirical Tuning

After setting initial values, drive the robot in a known straight line and a known rotation. Compare predicted vs actual position. Adjust:

1. **Straight line off?** → Tune `wheelRadius` and `gearRatio`
2. **Turns off?** → Tune `trackWidth`
3. **Both off?** → Tune `encoderTicksPerRev`

---

## Next Steps

- [Sensor Setup](Sensor-Setup.md) — Configure distance sensors
- [Tuning Guide](Tuning-Guide.md) — Fine-tune MCL and RAMSETE
