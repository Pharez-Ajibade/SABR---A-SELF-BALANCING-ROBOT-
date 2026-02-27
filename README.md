# SABR — Self-Balancing Robot

<img width="1280" height="720" alt="1" src="https://github.com/user-attachments/assets/e7bdf0b2-8088-4e61-ac17-db4423803b3b" />

### A LIGIENCE Project

SABR is a self-balancing robot built entirely from scratch using core engineering principles, control theory, and continuous iteration. This is not a copy of a tutorial — every design decision was made from the ground up using physics, math, and a lot of trial and error.

This repo contains all project files across multiple build versions,including a stepper motor version, DC motor version, and servo motor version.

---

## 📌 Project Description

SABR is modelled after an **inverted pendulum on a cart** — one of the most classic unstable systems in control theory. The robot uses an IMU to measure its tilt angle, feeds that into a PID control loop, and drives its motors to correct its position in real time at approximately 200Hz.

The goal of this project was to explore:
- Real-world control theory and system modelling
- Sensor fusion for accurate angle estimation
- Motor dynamics and actuator selection
- PCB design and embedded systems development

---

## Development Roadmap

### Phase 1 — Research & Physics
- Studied the inverted pendulum system model
- Derived equations of motion using torque, inertia, and angular acceleration
- Researched closed-loop control systems best suited for the build
- Key insight: longer rod = slower fall (inertia scales with L², torque scales with L)

### Phase 2 — Component Selection & Breadboard Prototyping
- Selected ESP32 as the microcontroller for dual-core capability and 240MHz clock speed
- Selected MPU6050 for IMU (3-axis accelerometer + gyroscope)
- Implemented and tested Complementary Filter and Kalman Filter for sensor fusion
- Tested stepper motor control with TMC2208 drivers using dual-core threading on ESP32
- Plotted and compared gyro drift, accelerometer noise, and filter outputs via serial oscilloscope

### Phase 3 — SABR V1 (Stepper Motor Version)
- 3D modelled and printed frame inspired by Aaed Musa's Impulse robot
- DIY rubber band wheels for grip
- Hand-soldered circuit board
- **Result:** Motors skipped steps, insufficient torque, thermal shutdown of drivers
- **Lesson:** Always calculate required torque before selecting motors

### Phase 4 — SABR V2 (Simplified Stepper Version)
- Redesigned to a shorter, simpler frame to reduce torque requirements
- Same circuit board, revised code
- **Result:** Closer to balancing but still inconsistent — heavy oscillations, drifting

### Phase 5 — PCB Design & New Motor Iterations
- Designed first custom PCB using KiCad, manufactured by PCBWay
- Switched to geared DC motors with TB6612FNG then L298N motor driver
- **Issue:** PWM dead zone (0–40%) meant motors didn't respond to small PID corrections
- Switched to 20kg·cm continuous rotation servos
- **Issue:** Too slow compared to high-end servos used in reference builds
- **Lesson:** Actuator speed and response time are just as critical as control theory

### Phase 6 — The Breakthrough
- Discovered the root cause of all balancing failures: **lack of friction**
- DIY wheels had insufficient grip on hard surfaces (table, tile floor)
- Robot successfully balanced on carpet
- Revised Kalman filter tuning: increased gyro trust from ~75% to over 95%
- Result: Significantly smoother angle readings and robot response

### Phase 7 — Tuning & Final Version
- Tuned PID starting with Kp, then Kd for dampening, then Ki for steady-state error
- Tested cascaded PI controllers (angular + position)
- Tested dynamic microstepping (1/16 step under 0.5° error)
- Tested variable Kp at different error angles
- Final implementation: Angular PID controller with gyro-based D term
- **Result:** SABR balances stably — position/velocity controller pending for V2

---

## 🧰 Components List

### Microcontroller
| Component | Details |
|-----------|---------|
| ESP32 Dev Module | 240MHz dual-core, Wi-Fi + Bluetooth |

### IMU / Sensors
| Component | Details |
|-----------|---------|
| MPU6050 | 3-axis accelerometer + 3-axis gyroscope |

### Motors & Drivers — Final Version
| Component | Details |
|-----------|---------|
| NEMA 17 Stepper Motors (x2) | Primary drive motors |
| TMC2208 Stepper Drivers (x2) | Silent driver with StealthChop, 1/8 microstepping |

> ⚠️ **Other motor iterations tested:** Geared DC motors (with TB6612FNG and L298N drivers) and 20kg·cm continuous rotation servos were both tested in earlier versions. Both were ultimately limited by dead zones and speed respectively. See the video for the full breakdown.

### Power
| Component | Details |
|-----------|---------|
| LiPo Battery | Main power source |
| Buck Converter | Steps down battery voltage for electronics |

### Mechanical
| Component | Details |
|-----------|---------|
| 3D Printed Frame | Custom designed, multiple versions |
| DIY Rubber Band Wheels | Custom modelled with grooves for rubber bands |
| M3 Hardware | Screws and standoffs for assembly |

### PCB
| Component | Details |
|-----------|---------|
| Custom PCB (KiCad) | Designed and manufactured via PCBWay |
| JST Connectors | Power input connectors |
| THT Components | Through-hole components throughout |

---

## 📐 Torque Calculation — Motor Sizing Guide

Before selecting your motors, calculate the torque required to balance your robot. Skipping this step is the single biggest mistake you can make — and yes, this was learned the hard way.

### Formula

```
τ = m · g · L · sin(θ)
```

| Variable | Description |
|----------|-------------|
| `τ` | Required torque (N·m) |
| `m` | Total mass of the robot (kg) |
| `g` | Gravitational acceleration (9.81 m/s²) |
| `L` | Distance from wheel axle to robot's centre of gravity (m) |
| `θ` | Maximum tilt angle the robot must recover from (degrees) |

### How to Apply It

1. Estimate `L` as the height from the motor axle to the top of your robot — this is a deliberate overestimate and that's fine
2. Set `θ` to at least **30°** as your minimum recovery angle
3. Calculate `τ`, then **multiply by 1.5× to 2×** as a safety margin — never run at minimum spec
4. Use this value to shortlist motors, then cross-check stall torque vs rated speed at your operating voltage

### Critical Nuances

> **Torque scales with L, but inertia scales with L²**
> A longer robot falls more slowly (easier to catch) but demands significantly more torque to correct. The two don't cancel out — inertia always wins at extreme lengths.

> **Height vs Speed tradeoff**
> A shorter robot requires less torque but falls much faster, demanding higher motor speeds and faster control loop response. A longer robot is more forgiving on speed but punishing on torque.

> **Motor dead zones kill small corrections**
> For DC motors especially, any PWM range where the motor doesn't move (typically 0–40%) creates a dead zone. When the PID output is small (near balance), the motors simply won't respond — and the robot falls. This cannot be tuned away. It is a hardware problem.

> **Stepper motor thermal limits**
> Steppers draw near-full current even at standstill. Under aggressive PID correction, drivers will heat up fast and can thermal-shutdown mid-balance. Size your drivers and cooling accordingly.

> **You cannot out-tune a hardware deficiency**
> Increasing Kp to compensate for insufficient torque only causes jitter and oscillation. If the motor can't physically move fast enough or with enough force to catch the robot, no controller gain will fix it.

---

## 🖼️ Visuals

### Circuit Diagram
![Circuit Diagram](images/circuit_diagram.png)

![DESIGN OVER VIEW](https://github.com/user-attachments/assets/749dd470-18d6-4bea-b9b9-0d612bd1ae36)


> *Full schematic including ESP32, MPU6050, TMC2208 drivers, buck converter and power connections*

---

### PCB Design
![PCB Layout](images/pcb_layout.png)

<img width="1919" height="1079" alt="kiCAD PCB SCHEMATIC SCREENSHOT " src="https://github.com/user-attachments/assets/890f8246-96de-4791-8f6b-2989a810586f" />

<img width="1919" height="1079" alt="kiCAD PCB 3D SCREENSHOT 2" src="https://github.com/user-attachments/assets/50a8cf3b-146d-4f15-92de-288d9043d847" />

<img width="1919" height="1079" alt="kiCAD PCB 3D SCREENSHOT " src="https://github.com/user-attachments/assets/9cc9320f-24fc-4a30-923e-5226349b9119" />

> *KiCad PCB layout — front and back copper layers with ground and 5V infill*

---

### 3D Model & Assembly
![3D Model](images/3d_model.png)

<img width="1919" height="1079" alt="SABR TEST PLATFORM 3d model " src="https://github.com/user-attachments/assets/63c6d550-bd0b-49df-bade-e6a6e35d8a53" />
<img width="1920" height="1080" alt="SABR TEST PLATFORM" src="https://github.com/user-attachments/assets/6242e579-1cdf-4a93-a96f-68bfb4f852ed" />

https://github.com/user-attachments/assets/52e14db4-9bfc-43c8-931d-ffd89795920e
> *SABR frame — full 3D model with motor mounts, wheel design and electronics bay*

<!-- Optional: Replace with an animated GIF of the 3D model rotating -->
<!-- ![3D Model Animation](images/3d_model_animation.gif) -->

---


### Software & Tools
- **Arduino IDE / PlatformIO** — for ESP32 firmware
- **KiCad** — PCB design
- **Fusion 360 / CAD software** — 3D frame modelling
- **Serial Oscilloscope** — for sensor and filter output visualisation

### Key Concepts
- **Inverted Pendulum Model** — system dynamics and equations of motion
- **Sensor Fusion** — Complementary Filter and Kalman Filter for stable angle estimation
- **PID Control** — Proportional, Integral, Derivative tuning for real-time balance
- **Dual-Core Threading (ESP32)** — separating step pulse timing from control loop calculations
- **Microstepping** — smoother stepper motor motion using 1/8 and 1/16 step modes

### Control Loop
- Sampling rate: ~200Hz
- IMU → Kalman Filter → Error Calculation → PID → Motor Output → Repeat
- D term based on gyroscope measurement (not error delta) for reduced jitter
- Gyro trust ratio: >95% in Kalman filter for smooth response

### Known Issues & Future Improvements (V2)
- [ ] Implement position/velocity controller to eliminate drift
- [ ] Proper silicone wheels for reliable grip on all surfaces
- [ ] Add power switch to PCB in next revision
- [ ] Explore bipedal walking version (BIPEDAL SABR)
- [ ] External wheel encoders for closed-loop position control
- [ ] Use of BLDC AND foc FOR MAIN TYRES 

---

## 🙏 Credits & References
- Inspired by Aaed Musa's Impulse Robot
- PCB manufactured by [PCBWay](https://www.pcbway.com)
- Mathematical references and resources linked in video description
- https://ctms.engin.umich.edu/CTMS/?example=InvertedPendulum&section=SystemModeling
- SENSOR FUSION
   -   https://youtu.be/7HVPjkWOrLE?si=l8aKTfFblnUTjnXL
   -   https://youtu.be/hQUkiC5o0JI?si=4qGIxBiCNVThZBNw
   -   https://youtu.be/RZd6XDx5VXo?si=hTg3l8ioiw1hPVCZ
- Accelerometer Roll and Pitch ; https://mwrona.com/posts/accel-roll-pitch/

---

*Part of the LIGIENCE robotics series — building real robots from first principles.*
