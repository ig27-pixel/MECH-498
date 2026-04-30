# Lab 1

Coordinate-transform and Phantom robot kinematics lab. Covers rotation matrices,
homogeneous transforms, screw and DH transforms, forward kinematics, and joint
actuation conversion for the Phantom haptic robot.

## Files

- `lab1.py`: student implementation for rotations, homogeneous transforms,
  screw/DH transforms, Phantom FK, and actuator-to-joint conversion.
- `lab1_utility.py`: provided drawing utilities for the Phantom robot.
- `general_utility.py`: shared validation and transform helpers.
- `run_path.py`: loads `path.yaml`, animates the robot path, and writes
  `phantom_video.gif`.
- `path.yaml`: joint path data for the animation.
- `phantom_video.gif`: pre-generated reference animation (overwritten on re-run).
- `498-2026-lab1.pdf`: original assignment handout.

## Run

From this folder:

```bash
python run_path.py
```

`run_path.py` expects `path.yaml` in the current working directory and saves the
animation as `phantom_video.gif`.

## Notes

- Dependencies: `PyKDL` (Python Kinematics and Dynamics Library), `numpy`,
  `matplotlib`, `yaml`.
- `lab1.py` is the file to inspect first when checking the transform math.
- Rerunning `run_path.py` overwrites the existing `phantom_video.gif`.

