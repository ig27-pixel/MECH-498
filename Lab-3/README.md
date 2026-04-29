# Lab 3

Inverse-kinematics and brush-path drawing lab built on the Fanuc robot stack.

## Files

- `fanuc_provided.py`: provided Fanuc model and drawing implementation.
- `fanuc_ik.cpython-312-x86_64-linux-gnu.so`: compiled IK helper used by the
  provided Fanuc implementation.
- `picasso.py`: student path-planning layer that converts brush-tip paths into
  end-effector poses and IK solutions.
- `basketball.yaml`, `prism.yaml`, `tetra.yaml`: path/shape data for drawing
  tasks.
- `robot_components.py`, `drawing_helper.py`, `general_utility.py`: shared robot
  support code.
- `test_script_student.py`: local runner for the Picasso/Fanuc path work.
- `498-2026-lab3.pdf`: original assignment handout.

## Run

From this folder:

```bash
python test_script_student.py
python picasso.py
```

`picasso.py` currently draws the `basketball.yaml` path from a fixed starting
configuration.

## Notes

- The key student logic is in `Picasso.get_ee_pose_from_brush()` and
  `Picasso.calculate_picasso_path()`.
- Path outputs are lists of `[q1, q2, q3, q4, q5, q6, color]`.
- The compiled IK library is platform-specific; use the course environment if
  imports fail locally.
