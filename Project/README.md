# Project

Final project workspace for **RoboRoll Coatings** — a custom 4-DOF serial robot
designed to paint flat vertical surfaces. The robot uses a prismatic base (Z
lift), two revolute shoulder joints, and a revolute wrist to reach two
perpendicular walls.

## Folder Layout

| Folder / File | Contents |
| --- | --- |
| `project intro 2026.pdf` | Original course project brief |
| `section_1_2_kinematics/` | Robot model, FK/IK, and DH parameter diagram |
| `section_3_demo/` | Two-wall painting trajectory demo and snapshot |
| `section_4_dynamics/` | Passive and controlled dynamics simulation and plots |
| `pitch_deck/` | HTML/PDF deck, plan notes, and mockup renders |
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

# Section 3 — two-wall painting animation
python Project/section_3_demo/project_demo.py

# Section 4 — dynamics simulation plots
python Project/section_4_dynamics/RoboRoll_dynamics.py

# Pitch deck — regenerate mockup images
python Project/pitch_deck/render_pitch_mockups.py
```

## Pitch Deck Files

- `pitch_deck/roboroll_pitch_deck.html`: browser-presentable slide deck.
- `pitch_deck/roboroll_pitch_deck.pdf`: PDF export (print the HTML using Chrome or Edge → Save as PDF).
- `pitch_deck/pitch_deck_plan.md`: slide strategy and speaker notes.

## Notes

- Joint angles are in **radians**; distances are in **millimeters**.
- `section_1_2_kinematics/RoboRoll.py` is the full visual/development model.
- `../base_robot.py` is the compact submission model with the same kinematic
  structure but no drawing dependencies.
- The Section 3 demo paints Wall 1 with the smiley face and Wall 2 with five
  color stripes.
