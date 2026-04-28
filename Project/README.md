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

This opens a matplotlib window and runs a Lab-3-style painting demonstration:

- a brush tool is attached to the end effector
- the RoboRoll arm paints a compact smiley on a reachable wall plane
- all brushes can remain visible while only the selected brush paints
- colored paint points are left behind in 3D space as the robot moves

The effect is intentionally similar to the `picasso.py` style demonstration in `Lab-3`, but adapted to the 4-DOF RoboRoll project robot and its more limited reachable wall geometry.

You can speed the demo up by adjusting the constructor settings inside
`project_demo.py`:

- `samples_per_segment`: fewer path samples between waypoints
- `max_joint_step_deg`: larger allowed IK interpolation step
- `frame_delay`: less time between rendered frames
- `paint_stride`: paint every Nth frame instead of every frame
- `show_all_brushes`: keep all brushes visible while only the selected one paints

## Notes

- The project robot model in `RoboRoll.py` has been aligned with the tested kinematic structure used by the submission model in `base_robot.py`.
- The autograder-facing implementation still lives at the repository root in `base_robot.py`.
