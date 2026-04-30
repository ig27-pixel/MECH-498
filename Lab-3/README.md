# Lab 3

Inverse-kinematics and brush-path drawing lab built on the Fanuc robot stack.
Students implement a path-planning layer that converts brush-tip trajectories
into IK joint solutions and drives the Fanuc robot to draw shapes.

## Files

- `fanuc_provided.py`: provided Fanuc model and drawing implementation.
- `fanuc_ik.cpython-312-x86_64-linux-gnu.so`: compiled IK helper (x86\_64 Linux
  only) used by the provided Fanuc implementation.
- `picasso.py`: student path-planning layer — converts brush-tip paths into
  end-effector poses and calls IK.
- `basketball.yaml`, `prism.yaml`, `tetra.yaml`: path/shape data for the
  drawing tasks.
- `robot_components.py`, `drawing_helper.py`, `general_utility.py`: shared robot
  support code (unchanged from Lab 2).
- `test_script_student.py`: local runner for the Picasso/Fanuc path work.
- `498-2026-lab3.pdf`: original assignment handout.

## Run

From this folder:

```bash
# run visual tests first
python test_script_student.py

# then run the full drawing demo
python picasso.py
```

`picasso.py` draws the `basketball.yaml` path from a fixed starting
configuration by default.

## Notes

- The two key student methods are `Picasso.get_ee_pose_from_brush()` and
  `Picasso.calculate_picasso_path()`.
- Path outputs are lists of `[q1, q2, q3, q4, q5, q6, color]`.
- The compiled IK library targets **x86\_64 Linux only**; run inside the
  course-provided Linux environment or WSL if imports fail.
