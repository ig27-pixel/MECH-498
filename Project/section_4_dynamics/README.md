# Section 4 — Dynamics Simulation

Physics-based simulation of the RoboRoll arm covering passive free-fall
(energy conservation check) and controlled motion (PD + gravity feedforward).

## Files

- `RoboRoll_dynamics.py`: full dynamics simulation — numerical integration
  with `scipy.integrate.solve_ivp`, energy plots, and torque/velocity plots.
- `dynamics_passive.png`: generated plot — Part 1 kinetic, potential, and
  total energy during passive (zero-torque) free fall.
- `dynamics_controlled.png`: generated plot — Part 2 joint torques and
  velocities under PD + gravity-feedforward control to a painting target.

## Run

From the repository root (`MECH-498`):

```bash
python Project/section_4_dynamics/RoboRoll_dynamics.py
```

Overwrites both PNG files on each run.

## Simulation Details

**Part 1 — Passive motion**

The robot is released from an off-home configuration with zero torque. Total
mechanical energy (KE + PE) must be conserved; the plot verifies this.

**Part 2 — Controlled motion**

A PD controller with gravity feedforward drives the arm from home to a
representative painting target. Joint torques and velocities are plotted over
the manoeuvre time.

## Notes

- All dynamics math uses SI units (m, kg, N·m, s); the robot model is defined
  in millimeters and divided by 1000 internally.
- The simulation treats J2/J3/J4 as operating in a vertical plane whose
  normal rotates with J1 (base yaw).
- `dynamics_passive.png` and `dynamics_controlled.png` are used directly by
  slide A6 in the pitch deck.
