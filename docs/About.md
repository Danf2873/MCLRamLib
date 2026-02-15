# About

## Project Goals

MCLRamLib was built to solve a common problem in VEX V5 robotics: **odometry alone isn't accurate enough for competition-level autonomous routines.**

Wheel slip, gear backlash, and accumulated integration errors cause pure-odometry solutions to drift several inches over the course of a 15-second autonomous period. MCLRamLib addresses this by fusing multiple data sources through a Bayesian probabilistic framework.

---

## How It Works

```
┌──────────────┐     ┌──────────────┐
│   Odometry   │     │  MCL Filter  │
│ (IMU + Enc)  │     │ (Particles)  │
└──────┬───────┘     └──────┬───────┘
       │                    │
       │  Prediction   Sensor Update
       │  (Motion)     (Distance)
       │                    │
       └────────┬───────────┘
                │
         ┌──────┴──────┐
         │ Bayes Filter │
         │ (Confidence  │
         │  Weighted)   │
         └──────┬──────┘
                │
         ┌──────┴──────┐
         │ Fused Pose  │
         │ (Best Est.) │
         └─────────────┘
```

1. **Odometry** tracks incremental motion from motor encoders and the IMU
2. **MCL** maintains a cloud of "particles" (hypothetical robot positions) and refines them using distance sensor measurements against the known field map
3. **Bayes Filter** blends the two estimates, weighting by MCL confidence (particle spread)

The result is a robust position estimate that:
- Is accurate even when wheels slip
- Self-corrects when passing near walls
- Degrades gracefully when sensors are occluded

---

## Technology

- **Language:** C++17
- **Platform:** PROS 4.1.1 for VEX V5
- **Algorithms:** Monte Carlo Localization, RAMSETE nonlinear control
- **Design:** Modular, header/source pairs, namespace-scoped

---

## License

This project is open-source. See the repository for license details.

---

## Credits

Built for the VEX V5 Robotics Competition community.
