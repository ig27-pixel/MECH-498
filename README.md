# MECH 498

Course workspace for MECH 498 robotics labs and the final RoboRoll Coatings
painting-robot project.

## Repository Layout

- `Lab-0`: Python warm-up using retirement-investment classes.
- `Lab-1`: coordinate transforms, Phantom kinematics, and path animation.
- `Lab-2`: Fanuc forward/inverse kinematics and robot drawing utilities.
- `Lab-3`: Fanuc IK path following with brush/tool-offset drawing tasks.
- `Lab-4`: RRBot and RobStudent simulation, dynamics, trajectory, and control work.
- `Project`: final RoboRoll 4-DOF painting robot, organized by project section.
- `base_robot.py`: compact RoboRoll model for project submission.
- `robot_config.yaml`: RoboRoll metadata, workspace bounds, DH values, and joint limits.

## Environment

The code is Python-based and uses course-provided helper files. Common
dependencies across the labs include:

- `numpy`
- `matplotlib`
- `PyYAML`
- `PyKDL` for Lab 1
- the compiled course helper libraries in `Lab-3` and `Lab-4`

Run scripts from the folder named in each lab README unless a command says
otherwise. Several scripts open matplotlib windows.

## Useful Entry Points

| Folder | Command |
| --- | --- |
| repository root | `python Lab-0/investment_student.py` |
| `Lab-1` | `python run_path.py` |
| `Lab-2` | `python test_script_student.py` |
| `Lab-3` | `python test_script_student.py` |
| `Lab-4` | `python test_rr_bot.py` |
| `Lab-4` | `python test_rob.py 1` |
| repository root | `python Project/section_3_demo/project_demo.py` |

## Notes

- Assignment PDFs are kept beside the corresponding lab/project files.
- `__pycache__` and `*.Zone.Identifier` files are generated local artifacts.
- The final project implementation is split between the interactive model in
  `Project/section_1_2_kinematics/RoboRoll.py` and the submission model in `base_robot.py`.
