# Project

Final project workspace for **RoboRoll Coatings** — a custom 4-DOF serial robot
designed to paint flat vertical surfaces. The robot uses four revolute joints:
a base yaw (J1), shoulder pitch (J2), elbow pitch (J3), and wrist pitch (J4),
with a fixed 500 mm column and a 200 mm paint nozzle offset.

## Folder Layout

| Folder / File | Contents |
| --- | --- |
| `project intro 2026.pdf` | Original course project brief |
| `section_1_2_kinematics/` | Robot model, FK/IK, and DH parameter diagram |
| `section_3_demo/` | Four-wall painting trajectory demo and snapshot |
| `section_4_dynamics/` | Passive and controlled dynamics simulation and plots |
| `pitch_deck/` | Investor pitch deck slides and mockup images |
| `shared/` | Reusable drawing, utility, joint, link, and brush helpers |

## Related Root Files

- `../base_robot.py`: compact `BaseCustomRobot` submission model.
- `../robot_config.yaml`: robot name, workspace bounds, DH dimensions, and
  joint limits used by the submission model.

## Run

From the repository root (`MECH-498`):

```bash
# Section 1/2 — regenerate DH parameter diagram
python Project/section_1_2_kinematics/dh_graphic.py

# Section 3 — four-wall painting animation
python Project/section_3_demo/project_demo.py

# Section 4 — dynamics simulation plots
python Project/section_4_dynamics/RoboRoll_dynamics.py
```

## Notes

- Joint angles are in **radians**; distances are in **millimeters**.
- `section_1_2_kinematics/RoboRoll.py` is the full visual/development model.
- `../base_robot.py` is the compact submission model with the same kinematic
  structure but no drawing dependencies.
- The Section 3 demo paints four walls: Wall 1 (X = +900 mm) with a smiley
  face, and Walls 2–4 each with five horizontal colour stripes.
