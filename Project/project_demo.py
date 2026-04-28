import math
import time

import numpy as np

from RoboRoll import RoboRoll


def build_demo_path() -> list[np.ndarray]:
    """Create a short sequence of smooth robot poses for visualization."""
    waypoints = [
        np.array([0.0, 0.0, 0.0, 0.0], dtype=float),
        np.array([0.020, 0.010, 0.012, -0.010], dtype=float),
        np.array([0.035, -0.012, 0.020, -0.015], dtype=float),
        np.array([0.010, -0.018, 0.030, 0.006], dtype=float),
        np.array([-0.020, 0.006, 0.016, 0.010], dtype=float),
        np.array([-0.035, 0.018, -0.008, 0.012], dtype=float),
        np.array([0.0, 0.0, 0.0, 0.0], dtype=float),
    ]

    frames = []
    samples_per_segment = 25
    for start, end in zip(waypoints[:-1], waypoints[1:]):
        for alpha in np.linspace(0.0, 1.0, samples_per_segment, endpoint=False):
            blend = alpha * alpha * (3.0 - 2.0 * alpha)
            frames.append(start + blend * (end - start))
    frames.append(waypoints[-1])
    return frames


def print_pose_summary(robot: RoboRoll, joint_angles: np.ndarray) -> None:
    ee = robot.calculate_fk(joint_angles)
    position = ee[:3, 3]
    print(
        "q =",
        np.array2string(joint_angles, precision=4, suppress_small=True),
        " -> ee xyz =",
        np.array2string(position, precision=2, suppress_small=True),
    )


def main() -> None:
    robot = RoboRoll(drawing_enabled=True)
    path = build_demo_path()

    print("RoboRoll visualization demo")
    print(f"Frames: {len(path)}")
    print("Showing a short FK/IK-ready motion sequence for Part 3.")

    for joint_angles in path:
        robot.draw_robot(joint_angles)
        time.sleep(0.03)

    print_pose_summary(robot, path[-1])
    print("Demo complete. Close the matplotlib window when finished.")

    try:
        import matplotlib.pyplot as plt

        plt.show()
    except Exception:
        pass


if __name__ == "__main__":
    main()
