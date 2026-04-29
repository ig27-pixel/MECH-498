"""Export the Section 3 RoboRoll demo animation as a GIF.

Run from the repository root:
    python Project/section_3_demo/export_demo_gif.py
"""

import os

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from PIL import Image

from project_demo import RoboRollRoomDemo


DEMO_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_PATH = os.path.join(DEMO_DIR, "roboroll_demo.gif")


def export_gif() -> None:
    robot = RoboRollRoomDemo(
        drawing_enabled=True,
        frame_delay=0.0,
        samples_per_segment=5,
    )
    robot.fig.set_size_inches(6.4, 6.4)
    robot.fig.set_dpi(100)

    path = robot.build_demo_path()
    frames = []
    keep_every = 2

    for index, (joint_angles, color, should_paint, wrist_angle) in enumerate(path):
        robot.draw_demo_pose(
            joint_angles,
            color=color,
            leave_paint=should_paint,
            wrist_visual_angle=wrist_angle,
        )

        if index % keep_every == 0 or index == len(path) - 1:
            robot.fig.canvas.draw()
            width, height = robot.fig.canvas.get_width_height()
            rgba = memoryview(robot.fig.canvas.buffer_rgba())
            frame = Image.frombuffer(
                "RGBA",
                (width, height),
                rgba,
                "raw",
                "RGBA",
                0,
                1,
            ).convert("P", palette=Image.Palette.ADAPTIVE)
            frames.append(frame.copy())

    frames[0].save(
        OUT_PATH,
        save_all=True,
        append_images=frames[1:],
        duration=55,
        loop=0,
        optimize=True,
    )
    plt.close(robot.fig)

    print(f"Saved {OUT_PATH}")
    print(f"Rendered {len(frames)} GIF frames from {len(path)} demo poses")


if __name__ == "__main__":
    export_gif()
