# Lab 2

Fanuc robot kinematics lab introducing the shared robot-component and drawing
framework used throughout the rest of the course.

## Files

- `fanuc.py`: student Fanuc model — workspace setup, DH parameters, forward
  kinematics (FK), inverse kinematics (IK), and drawing hooks.
- `robot_components.py`: reusable `Joint`, `Link`, and brush/component drawing
  primitives.
- `drawing_helper.py`: 3D frame, link, and point drawing support.
- `general_utility.py`: transform and validation helpers.
- `test_script_student.py`: local visual test runner for Fanuc drawing and FK.
- `498-2026-lab2.pdf`: original assignment handout.

## Run

From this folder:

```bash
python test_script_student.py
```

The script creates a `Fanuc` instance and draws several test configurations in a
matplotlib 3D window.

## Notes

- Joint angles are in **radians**; distances are in **millimeters**.
- `fanuc.py` is the only file you need to edit.
- `robot_components.py` and `drawing_helper.py` carry forward unchanged into
  Lab 3 — treat them as a reference when debugging robot visuals.
