# Development Journey

## Phase 1 — Physics & Modelling

* Inverted pendulum system
* Derived equations of motion

## Phase 2 — Prototyping

* ESP32 + MPU6050
* Complementary & Kalman filters tested

## Phase 3 — Stepper V1

* Torque issues, overheating

## Phase 4 — Stepper V2

* Improved but unstable

## Phase 5 — Motor Experiments

* DC motors → dead zone issues
* Servos → too slow

## Phase 6 — Breakthrough

* Root issue: wheel friction
* Successful balancing on carpet
* Kalman tuning improved stability

## Phase 7 — Final Tuning

* PID tuning (Kp → Kd → Ki)
* Gyro-based D term
* Stable balancing achieved

## Key Lessons

* Hardware matters more than tuning
* Friction is critical
* Motor response speed is everything
