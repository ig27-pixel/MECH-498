# Project

Final RoboRoll Coatings project workspace for a custom 4-DOF painting robot.

## Files

- `RoboRoll.py`: project robot class with FK, IK, joint limits, nozzle frame
  support, and matplotlib visualization.
- `project_demo.py`: Section 3 two-wall painting demonstration — Wall 1 (X = 900 mm)
  with five horizontal colour stripes, Wall 2 (Y = 900 mm) with a smiley face
  (outline, eyes, smile arc).
- `RoboRoll_dynamics.py`: Section 4 dynamics simulation — passive motion (energy
  conservation) and PD + gravity-feedforward controlled motion; saves
  `dynamics_passive.png` and `dynamics_controlled.png`.
- `robot_components.py`: reusable joint, link, brush, and drawing primitives.
- `drawing_helper.py`: 3D plotting helpers.
- `general_utility.py`: validation and transform helpers.
- `dh_graphic.py`: generates `dh_parameters.png` — Modified DH parameter diagram
  and table.
- `dh_parameters.png`: saved DH-parameter illustration (Section 1).
- `dynamics_passive.png`: saved energy-conservation plot (Section 4).
- `dynamics_controlled.png`: saved PD-controlled trajectory plot (Section 4).
- `project intro 2026.pdf`: original project handout.

## Related Root Files

- `../base_robot.py`: compact `BaseCustomRobot` implementation for project
  submission.
- `../robot_config.yaml`: robot name, workspace bounds, DH-style dimensions,
  and joint limits used by the submission model.

## Run

From the Project folder:

```bash
# Section 3 — two-wall painting animation
python project_demo.py

# Section 4 — dynamics plots (saves PNGs automatically)
python RoboRoll_dynamics.py

# Section 1 — regenerate DH parameter diagram
python dh_graphic.py
```

The demo opens a matplotlib window, places the robot at home, renders a 2000 mm
room box, and paints Wall 1 with five colour stripes followed by a smiley face
on Wall 2.

## Demo Tuning

Constructor parameters in `RoboRollRoomDemo` (bottom of `project_demo.py`):

- `frame_delay`: pause in seconds between rendered frames (default `0.005`).
- `samples_per_segment`: smoothstep interpolation density between IK waypoints
  (default `7`).

## Notes

- Joint angles are in radians; distances are in millimeters.
- `RoboRoll.py` is the visual/development model; `../base_robot.py` is the
  compact submission model that mirrors the same kinematic structure.
- The smiley face uses face-circle arc transits between features so the robot
  never cuts a diagonal line across the face panel.
