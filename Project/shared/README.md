# Shared — Common Utilities

Reusable drawing and math helpers imported by the robot model, demo, and
dynamics scripts across the project.

## Files

- `robot_components.py`: `Brush`, `Link`, `Joint`, and `RobotPayload` classes —
  handles DH transforms, 3D drawing artists, and paint dot rendering.
- `drawing_helper.py`: low-level matplotlib 3D primitives — `FrameDrawing`,
  `LinkDrawing`, and `PointDrawing` for axis frames, arm links, and markers.
- `general_utility.py`: transform helpers (`yawT`, `pitchT`, `rollT`),
  homogeneous matrix validation, and shared math utilities.

## Usage

Scripts in other sections add this folder to `sys.path` at runtime:

```python
import os, sys
sys.path.append(os.path.join(PROJECT_DIR, "shared"))
from robot_components import Joint, Link, Brush
```

## Notes

- Do not run these files directly — they are support libraries, not entry points.
- Paint dot size in the demo is controlled by `markersize` in
  `robot_components.py` (`Brush.paint`, line ~227).
- Joint angles passed to DH transform methods are in **radians**; link
  dimensions are in **millimeters**.
