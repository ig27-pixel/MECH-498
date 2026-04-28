# Project

Final project workspace for the RoboRoll Coatings painting robot.

## Main Files

- `RoboRoll.py`: project robot class with FK, IK, and visualization support.
- `project_demo.py`: separate Part 3 demonstration / visualization entry point.
- `robot_components.py`: reusable joint / link rendering primitives.
- `drawing_helper.py`: 3D plotting helpers.
- `general_utility.py`: validation and helper functions.
- `project intro 2026.pdf`: original project handout.

## Part 3 Demonstration

Run the dedicated demo script from the repository root:

```bash
python Project/project_demo.py
```

This opens a matplotlib window and animates a short RoboRoll motion sequence intended for the robot demonstration / visualization portion of the project.

## Notes

- The project robot model in `RoboRoll.py` has been aligned with the tested kinematic structure used by the submission model in `base_robot.py`.
- The autograder-facing implementation still lives at the repository root in `base_robot.py`.
