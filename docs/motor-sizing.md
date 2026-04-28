# Motor Sizing & Torque Calculation
Source: [CTMS - Inverted Pendulum Model](https://ctms.engin.umich.edu/CTMS/?example=InvertedPendulum&section=SystemModeling)

## Formula

`τ = m · g · L · sin(θ)`

## Variables

- **τ** - Torque (N·m)
- **m** - Mass of the robot (kg)
- **g** - Gravitational acceleration (9.81 m/s²)
- **L** - Distance from wheel axle to center of mass (m)
- **θ** - Tilt angle (degrees)

## Guidelines

- Use θ ≥ 30° as the minimum recovery angle.
- Multiply the calculated torque by a 1.5×–2× safety margin.
- Never select motors at the minimum torque specification.

## Key Insights

### Torque vs Inertia

- **Torque ∝ L** - Torque increases with the distance from the pivot.
- **Inertia ∝ L²** - Longer robots require more torque to counterbalance increased inertia.

### Dead Zones

DC motors often have a 0–40% PWM dead zone, where small PID corrections may fail to move the motor.

### Stepper Limitations

- Stepper motors can overheat due to high current draw at standstill.
- This can lead to thermal shutdown under continuous correction.

### Critical Rule

You cannot fix insufficient hardware with PID tuning. Ensure your motor provides enough torque and response speed.
