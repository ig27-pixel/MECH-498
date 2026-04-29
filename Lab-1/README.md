# Lab 1

Coordinate-transform and Phantom robot kinematics lab.

## Files

- `lab1.py`: student implementation for rotations, homogeneous transforms,
  screw/DH transforms, Phantom FK, and actuator-to-joint conversion.
- `lab1_utility.py`: provided drawing utilities for the Phantom robot.
- `general_utility.py`: shared validation and transform helpers.
- `run_path.py`: loads `path.yaml`, animates the robot path, and writes
  `phantom_video.gif`.
- `path.yaml`: joint path data for the animation.
- `phantom_video.gif`: generated/reference animation output.
- `498-2026-lab1.pdf`: original assignment handout.

## Run

From this folder:

```bash
python run_path.py
```

`run_path.py` expects `path.yaml` in the current working directory and saves the
animation as `phantom_video.gif`.

## Notes

- This lab imports `PyKDL`, `numpy`, `matplotlib`, and `yaml`.
- `lab1.py` is the file to inspect first when checking the transform math.
- The GIF is generated output; rerunning the script may overwrite it.

