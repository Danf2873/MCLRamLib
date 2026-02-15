# Tuning Guide

This page covers how to tune every adjustable parameter in the library for optimal competition performance.

---

## Overview

There are three systems to tune:

1. **MCL (Particle Filter)** — How accurately the robot corrects its position
2. **RAMSETE Controller** — How accurately the robot follows paths
3. **Bayes Filter** — How the odometry and MCL estimates are blended

---

## 1. MCL Parameters

Located in `mcl.hpp` as private members of `MCLLocalizer`:

### Particle Count

```cpp
MCLLocalizer(int numParticles = 200);
```

| Value | Trade-off |
|---|---|
| `100` | Fast but less accurate. Good for limited CPU budget |
| `200` | **Recommended default.** Good balance |
| `500` | Very accurate but uses more CPU. Consider for finals |

> [!TIP]
> The V5 brain can comfortably handle 200-300 particles at 100Hz. Monitor your loop timing with `pros::millis()` to ensure it stays under 10ms.

### Motion Noise (α parameters)

```cpp
double alpha1 = 0.05;  // rotation noise from rotation
double alpha2 = 0.05;  // rotation noise from translation
double alpha3 = 0.05;  // translation noise from translation
double alpha4 = 0.05;  // translation noise from rotation
```

These control how much the particle filter "spreads" during motion:

| Symptom | Fix |
|---|---|
| Particles converge too slowly | Decrease alpha values (0.02-0.03) |
| Particles diverge / lose track | Increase alpha values (0.08-0.15) |
| Robot slides a lot (omni wheels) | Increase alpha3 and alpha4 |
| High-traction wheels | Decrease all alphas |

### Measurement Noise

```cpp
double sigmaDistance = 2.0;  // inches
```

| Value | Effect |
|---|---|
| `1.0` | Very trusting of sensor readings. Good for clean fields |
| `2.0` | **Default.** Tolerant of minor sensor noise |
| `5.0` | Very skeptical. Use near other robots / noisy environments |

---

## 2. RAMSETE Parameters

Located in `RamseteController` constructor:

```cpp
RamseteController controller(2.0, 0.7);  // b, zeta
```

### `b` — Convergence Gain

| Value | Effect |
|---|---|
| `1.0` | Gentle correction, slower convergence |
| `2.0` | **Default.** Good balance |
| `4.0` | Aggressive correction, may oscillate |

### `zeta` — Damping Ratio

| Value | Effect |
|---|---|
| `0.3` | Under-damped, oscillates around path |
| `0.7` | **Default.** Critically damped, smooth |
| `0.9` | Over-damped, very smooth but slow to correct |

> [!WARNING]
> `zeta` must be between 0 and 1. Values outside this range will cause unstable behavior.

### Tuning Process

1. Start with defaults (`b=2.0`, `zeta=0.7`)
2. If the robot oscillates around the path → increase `zeta`
3. If the robot takes too long to get back on path → increase `b`
4. If the robot overshoots corners → decrease `b` or increase `zeta`

---

## 3. Bayes Filter Parameters

### Confidence Scale

Located in `mcl.cpp` inside `getConfidence()`:

```cpp
double scale = 3.0;  // inches
```

This controls how particle spread maps to MCL confidence:

| Value | Effect |
|---|---|
| `1.0` | Very strict. MCL needs tight particle clusters to be trusted |
| `3.0` | **Default.** Balanced trust threshold |
| `6.0` | Lenient. MCL is trusted even with some particle spread |

### How Fusion Works

`getFusedPose()` computes:

```
fusedPose = confidence × mclPose + (1 - confidence) × odomPose
```

- **Confidence ≈ 1.0**: Particles are tightly clustered → MCL is very confident → fused pose = MCL pose
- **Confidence ≈ 0.5**: Moderate particle spread → equal blend of both
- **Confidence ≈ 0.0**: Particles are scattered → MCL is unreliable → fused pose = odometry

### Telemetry

Monitor the confidence in your update loop:

```cpp
double conf = manager->getMCLConfidence();
pros::lcd::print(1, "MCL Conf: %.0f%%", conf * 100.0);
```

If confidence is consistently below 30%, check:
- Are your distance sensors getting clean readings?
- Is your field map accurate?
- Are sensor offsets measured correctly?

---

## Sensor-Specific Tuning

### DistanceSensorConfig

```cpp
manager->addSensor({
    &sensor,
    {5, 0, 0},   // offset
    2.0,          // minRange (inches)
    60.0,         // maxRange (inches)
    50            // minConfidence (0-100)
});
```

| Parameter | Symptom | Adjustment |
|---|---|---|
| `minRange` | False readings when too close to walls | Increase to 3-5" |
| `maxRange` | Noisy readings at distance | Decrease to 50-60" |
| `minConfidence` | Robot nearby causes MCL errors | Increase to 70-80 |

---

## Recommended Starting Values

| Parameter | Competition Default |
|---|---|
| Particles | `200` |
| α1-α4 | `0.05` |
| σ_distance | `2.0` |
| RAMSETE b | `2.0` |
| RAMSETE ζ | `0.7` |
| Bayes scale | `3.0` |
| Sensor minConf | `50` |

---

## Next Steps

- [API Reference](API-Reference.md) — Full class and method reference
