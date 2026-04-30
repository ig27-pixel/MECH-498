# MECH 498

Course workspace for MECH 498 robotics labs and the final RoboRoll Coatings
painting-robot project.

## Repository Layout

| Folder / File | Contents |
| --- | --- |
| `Lab-0/` | Python warm-up using retirement-investment classes |
| `Lab-1/` | Coordinate transforms, Phantom kinematics, and path animation |
| `Lab-2/` | Fanuc forward/inverse kinematics and robot drawing utilities |
| `Lab-3/` | Fanuc IK path following with brush/tool-offset drawing tasks |
| `Lab-4/` | RRBot and RobStudent simulation, dynamics, trajectory, and control |
| `Project/` | Final RoboRoll 4-DOF painting robot, organized by project section |
| `base_robot.py` | Compact RoboRoll model for project submission |
| `robot_config.yaml` | RoboRoll metadata, workspace bounds, DH values, and joint limits |

## Environment

- **Python 3.12** (required for the compiled `.so` helpers in Lab-3 and Lab-4)
- `numpy`, `matplotlib`, `PyYAML`
- `PyKDL` (Kinematics and Dynamics Library Python bindings) — Lab 1 only
- Compiled course helper libraries — Lab-3 and Lab-4 only

> **Platform note:** the precompiled `.so` libraries in Lab-3 and Lab-4 target
> Linux (x86\_64 and aarch64). Run those labs inside a course-provided Linux
> environment or WSL if you are on Windows or macOS.

Run scripts from the folder named in each lab README unless a command says
otherwise. Several scripts open matplotlib windows.

## Useful Entry Points

| Run from | Command | What it does |
| --- | --- | --- |
| Repository root | `python Lab-0/investment_student.py` | Retirement scenario results and plots |
| `Lab-1/` | `python run_path.py` | Phantom robot path animation → `phantom_video.gif` |
| `Lab-2/` | `python test_script_student.py` | Fanuc FK drawing in a 3D window |
| `Lab-3/` | `python test_script_student.py` | Fanuc IK visual tests |
| `Lab-4/` | `python test_rr_bot.py` | RRBot free-motion and setpoint-control |
| `Lab-4/` | `python test_rob.py 1` | RobStudent trajectory case 1 simulation |
| Repository root | `python Project/section_3_demo/project_demo.py` | Two-wall painting animation |

## Notes

- Assignment PDFs are kept beside the corresponding lab/project files.
- The final project implementation is split between the interactive model in
  `Project/section_1_2_kinematics/RoboRoll.py` and the submission model in
  `base_robot.py`.
