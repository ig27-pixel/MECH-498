# Section 3 — Two-Wall Painting Demo

Interactive 3D animation of RoboRoll painting across two perpendicular walls
using planned IK paths and smoothstep-interpolated waypoints.

## Files

- `project_demo.py`: main demo — builds the full two-wall painting path and
  animates it in a live matplotlib 3D window.
- `export_demo_gif.py`: headless version that captures frames and saves the
  animation as `roboroll_demo.gif`.
- `roboroll_demo.gif`: pre-generated reference animation.
- `roboroll_demo_snapshot.png`: still image of the completed demo scene.

## Run

From the repository root (`MECH-498`):

```bash
# Live interactive animation
python Project/section_3_demo/project_demo.py

# Export animation to GIF (no display required)
python Project/section_3_demo/export_demo_gif.py
```

## Demo Sequence

1. **Wall 2 (Y = 900 mm)** — five horizontal colour stripes painted top to
   bottom, alternating direction each pass.
2. **Wall 1 (X = 900 mm)** — smiley face: circular outline, two eyes, and a
   smile arc painted with IK-solved curved paths.

The demo stops after the last smile stroke; the window stays open until closed.

## Notes

- Joint angles are in **radians**; all distances are in **millimeters**.
- `RoboRollRoomDemo` inherits from `RoboRoll` (in `section_1_2_kinematics/`)
  and adds room geometry, brush painting, and path interpolation.
- Smoothstep interpolation blends between IK waypoints; lift motions prevent
  unwanted diagonal paint marks between strokes.
- Paint dot size is controlled by `markersize` in `shared/robot_components.py`.
