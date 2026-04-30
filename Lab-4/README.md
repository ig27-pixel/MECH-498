# Lab 4

Robot simulation, dynamics, trajectory generation, and controller lab.
Students implement joint-space trajectory generation and a torque controller
for a 3-DOF robot arm moving through pick-and-place waypoints.

## Files

- `RRBot.py`: two-link RRBot dynamics, energy plots, and setpoint-control tests.
- `RobBase.py`: base 3-DOF robot model, trajectory container, and shared
  properties.
- `RobSimulation.py`: provided simulation loop for the RobStudent task.
- `RobStudent.py`: student implementation of Cartesian waypoint trajectory
  generation and joint torque control.
- `rob_data.py`: data logging and diagnostic plotting helpers.
- `rob_dynamics.cpython-312-x86_64-linux-gnu.so`: compiled dynamics helper
  (x86\_64 Linux).
- `rob_dynamics.cpython-312-aarch64-linux-gnu-1.so`: compiled dynamics helper
  (aarch64 Linux).
- `robot_components.py`, `drawing_helper.py`, `general_utility.py`: shared robot
  drawing and math utilities.
- `test_rr_bot.py`: local RRBot free-motion and setpoint-control runner.
- `test_rob.py`: local RobStudent runner for one of three trajectory cases.
- `498_2026_lab4.pdf`: original assignment handout.

## Run

From this folder:

```bash
# Two-link RRBot dynamics and setpoint control
python test_rr_bot.py

# 3-DOF RobStudent trajectory cases (1, 2, or 3)
python test_rob.py 1
python test_rob.py 2
python test_rob.py 3
```

`test_rob.py` requires a trajectory number argument (1–3). It runs the
simulation and opens diagnostic matplotlib plots for visual debugging.

## Notes

- `RobStudent.create_rob_trajectory()` builds the 30-second joint-space
  trajectory through home → pickup → dropoff → home waypoints.
- `RobStudent.get_rob_torque()` computes the control torques used by the
  provided simulation loop.
- The compiled dynamics helpers support **x86\_64** and **aarch64** Linux;
  use the course-provided Linux environment or WSL if imports fail.
