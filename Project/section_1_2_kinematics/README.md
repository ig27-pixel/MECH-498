# Section 1 & 2 — Robot Model and Kinematics

Full visual model for the RoboRoll 4-DOF painting robot, covering Modified DH
frame assignment, forward kinematics, and inverse kinematics.

## Files

- `RoboRoll.py`: complete robot model — DH parameters, FK, IK, joint limits,
  and 3D matplotlib drawing support.
- `dh_parameters.png`: DH frame assignment diagram (pre-generated; submitted
  directly as the Section 1 graphic deliverable).

## Notes

- Joint angles are in **radians**; link dimensions are in **millimeters**.
- `RoboRoll` is the full visual/development model imported by the demo and
  dynamics scripts. The submission model lives at `../../base_robot.py`.
- The robot has 4 revolute joints: base yaw (J1), shoulder pitch (J2), elbow
  pitch (J3), and wrist pitch (J4), with a 200 mm paint nozzle offset.
- `dh_graphic.py` runs standalone — it does not import `RoboRoll.py`.
