# Project

Final RoboRoll Coatings project workspace for a custom 4-DOF painting robot.

## Folder Layout

- `project intro 2026.pdf`: original course project PDF.
- `section_1_2_kinematics/`: robot model, forward/inverse kinematics, and DH graphic.
- `section_3_demo/`: two-wall painting trajectory demonstration and demo snapshot.
- `section_4_dynamics/`: passive and controlled dynamics simulation plus saved plots.
- `pitch_deck/`: HTML, PDF, PPTX, plan notes, and generated pitch mockup renders.
- `shared/`: reusable drawing, utility, joint, link, and brush helper code.

## Related Root Files

- `../base_robot.py`: compact `BaseCustomRobot` implementation for project submission.
- `../robot_config.yaml`: robot name, workspace bounds, DH-style dimensions, and joint limits used by the submission model.

## Run

From the repository root (`MECH-498`):

```bash
# Section 1/2 - regenerate DH parameter diagram
python Project/section_1_2_kinematics/dh_graphic.py

# Section 3 - two-wall painting animation
python Project/section_3_demo/project_demo.py

# Section 4 - dynamics plots
python Project/section_4_dynamics/RoboRoll_dynamics.py

# Pitch deck - regenerate mockup images
python Project/pitch_deck/render_pitch_mockups.py

# Pitch deck - regenerate PowerPoint
python Project/pitch_deck/export_pitch_deck.py
```

## Pitch Deck Files

- `pitch_deck/roboroll_pitch_deck.html`: browser-presentable deck.
- `pitch_deck/roboroll_pitch_deck.pdf`: PDF export.
- `pitch_deck/roboroll_pitch_deck.pptx`: PowerPoint export.
- `pitch_deck/pitch_deck_plan.md`: slide strategy and speaker notes.

## Notes

- Joint angles are in radians; distances are in millimeters.
- `section_1_2_kinematics/RoboRoll.py` is the visual/development model.
- `../base_robot.py` is the compact submission model that mirrors the same kinematic structure.
- The Section 3 demo paints Wall 1 with five color stripes and Wall 2 with a smiley face.
