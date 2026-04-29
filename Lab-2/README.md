# Lab 2

Fanuc robot kinematics lab introducing the shared robot-component and drawing
framework used later in the course.

## Files

- `fanuc.py`: student Fanuc model with workspace setup, DH parameters, FK, IK,
  and drawing hooks.
- `robot_components.py`: reusable `Joint`, `Link`, and brush/component drawing
  primitives.
- `drawing_helper.py`: 3D frame, link, and point drawing support.
- `general_utility.py`: transform and validation helpers.
- `test_script_student.py`: local visual test runner for Fanuc drawing/FK.
- `498-2026-lab2.pdf`: original assignment handout.

## Run

From this folder:

```bash
python test_script_student.py
```

The script creates a `Fanuc` instance and draws several test configurations in a
matplotlib 3D window.

## Notes

- Joint angles are in radians in the Python API.
- Distances in the course robot models are in millimeters.
- Lab 3 reuses the same drawing/component style, so this folder is a useful
  reference when debugging later robot visuals.
