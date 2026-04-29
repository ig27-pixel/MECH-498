# Project

Final RoboRoll Coatings project workspace for a custom 4-DOF painting robot.

## Files

- `RoboRoll.py`: interactive project robot class with FK, IK, joint limits,
  nozzle frame support, and matplotlib visualization.
- `project_demo.py`: Part 3 room/painting demonstration using the RoboRoll
  visual model.
- `robot_components.py`: reusable joint, link, brush, and drawing primitives.
- `drawing_helper.py`: 3D plotting helpers.
- `general_utility.py`: validation and transform helpers.
- `dh_graphic.py`, `dh_parameters.png`: DH-parameter illustration support.
- `project intro 2026.pdf`: original project handout.

## Related Root Files

- `../base_robot.py`: compact `BaseCustomRobot` implementation for project
  submission.
- `../robot_config.yaml`: robot name, workspace bounds, DH-style dimensions,
  and joint limits used by the submission model.

## Run

From the repository root:

```bash
python Project/project_demo.py
```

The demo opens a matplotlib window, starts the robot from home, shows a 2000 mm
room box, and paints a compact multi-color path with the brush/nozzle attached
to the end effector.

## Demo Tuning

The main demo settings are in the `RoboRollRoomDemo` constructor call near the
bottom of `project_demo.py`:

- `frame_delay`: pause between rendered frames.
- `paint_stride`: paint every Nth rendered frame.
- `samples_per_segment`: interpolation density between waypoints.
- `show_all_brushes`: show all brush tools while only the selected brush paints.

## Notes

- `RoboRoll.py` is the visual/development model.
- `base_robot.py` is the compact submission model that mirrors the tested
  kinematic structure.
- Joint angles are in radians and distances are in millimeters.
