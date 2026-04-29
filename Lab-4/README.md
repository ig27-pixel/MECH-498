# Lab 4

Robot simulation, dynamics, trajectory generation, and controller lab.

## Files

- `RRBot.py`: two-link RRBot dynamics, energy plots, and setpoint-control tests.
- `RobBase.py`: base 3-DOF robot model, trajectory container, and shared
  properties.
- `RobSimulation.py`: provided simulation loop for the RobStudent task.
- `RobStudent.py`: student implementation of Cartesian waypoint trajectory
  generation and joint torque control.
- `rob_data.py`: data logging and diagnostic plotting helpers.
- `rob_dynamics.cpython-312-*.so`: compiled dynamics helpers for supported
  Linux platforms.
- `robot_components.py`, `drawing_helper.py`, `general_utility.py`: shared robot
  drawing and math utilities.
- `test_rr_bot.py`: local RRBot free-motion and setpoint-control runner.
- `test_rob.py`: local RobStudent runner for one of three trajectory cases.
- `498_2026_lab4.pdf`: original assignment handout.

## Run

From this folder:

```bash
python test_rr_bot.py
python test_rob.py 1
python test_rob.py 2
python test_rob.py 3
```

`test_rob.py` expects a trajectory number argument. It runs the simulation and
opens diagnostic matplotlib plots; it is a visual/debug runner, not a strict
pass/fail autograder.

## Notes

- `RobStudent.create_rob_trajectory()` builds the 30-second joint-space
  trajectory through home, pickup, dropoff, and home waypoints.
- `RobStudent.get_rob_torque()` computes the control torques used by the
  provided simulation loop.
- The compiled dynamics helpers are platform-specific; use the course-supported
  Python/Linux setup if imports fail locally.
