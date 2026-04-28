import time
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np

from RoboRoll import RoboRoll
from robot_components import Brush


class RoboRollPainter(RoboRoll):
    """Lab-3-style project demo with a brush that paints a spatial path."""

    def __init__(
        self,
        drawing_enabled: bool = True,
        samples_per_segment: int = 12,
        max_joint_step_deg: float = 6.0,
        frame_delay: float = 0.005,
        paint_stride: int = 1,
        show_all_brushes: bool = True,
    ):
        super().__init__(drawing_enabled=drawing_enabled)
        self.brush = Brush(self.ax, drawing_enabled=drawing_enabled)
        self.samples_per_segment = max(2, int(samples_per_segment))
        self.max_joint_step_deg = float(max_joint_step_deg)
        self.frame_delay = max(0.0, float(frame_delay))
        self.paint_stride = max(1, int(paint_stride))
        self.show_all_brushes = bool(show_all_brushes)

    def draw_painter(
        self,
        joint_angles: np.ndarray,
        color: int,
        leave_paint: bool = True,
    ) -> None:
        if not self._drawing_enabled:
            return

        self.brush.selection = int(color)
        super().draw_robot(joint_angles)
        self.brush.update_tool_frame(self.ee_frame)
        if self.show_all_brushes:
            self.brush.show_all()
        else:
            self.brush.show_enabled()
        if leave_paint:
            self.brush.paint(show_all_tools=self.show_all_brushes)
        plt.pause(0.00001)

    def get_ee_pose_from_brush_frame(
        self,
        brush_frame: np.ndarray,
        color: int,
    ) -> np.ndarray:
        self.brush.selection = int(color)
        return brush_frame @ np.linalg.inv(self.brush.selected_brush_frame_dh)

    def build_colored_joint_demo(self) -> List[Tuple[np.ndarray, int]]:
        waypoints = [
            (np.array([0.000, 0.000, 0.000, 0.000], dtype=float), 1),
            (np.array([0.022, 0.012, 0.012, -0.010], dtype=float), 1),
            (np.array([-0.016, 0.018, 0.016, -0.004], dtype=float), 2),
            (np.array([0.028, 0.004, -0.004, -0.012], dtype=float), 2),
            (np.array([-0.024, 0.014, 0.010, 0.008], dtype=float), 3),
            (np.array([0.014, -0.002, -0.010, 0.010], dtype=float), 3),
            (np.array([-0.020, 0.016, 0.018, -0.002], dtype=float), 4),
            (np.array([0.000, 0.000, 0.000, 0.000], dtype=float), 4),
        ]

        frames: List[Tuple[np.ndarray, int]] = []
        for (start, color), (end, _) in zip(waypoints[:-1], waypoints[1:]):
            for alpha in np.linspace(
                0.0,
                1.0,
                self.samples_per_segment,
                endpoint=False,
            ):
                blend = alpha * alpha * (3.0 - 2.0 * alpha)
                frames.append((start + blend * (end - start), color))
        frames.append(waypoints[-1])
        return frames

    def build_brush_targets(self) -> List[Tuple[np.ndarray, int]]:
        targets: List[Tuple[np.ndarray, int]] = []
        for joint_angles, color in self.build_colored_joint_demo():
            self.brush.selection = int(color)
            ee_frame = self.calculate_fk(joint_angles)
            brush_frame = ee_frame @ self.brush.selected_brush_frame_dh
            targets.append((brush_frame.copy(), color))
        return targets

    def calculate_demo_ik_path(
        self,
        starting_angles: np.ndarray,
    ) -> List[Tuple[np.ndarray, int]]:
        prev_angles = np.asarray(starting_angles, dtype=float).reshape(-1)
        output: List[Tuple[np.ndarray, int]] = []

        for brush_frame, color in self.build_brush_targets():
            ee_pose = self.get_ee_pose_from_brush_frame(brush_frame, color)
            success, q_new = self.calculate_ik(ee_pose, prev_angles)
            if not success:
                q_new = prev_angles.copy()

            max_diff_deg = np.degrees(np.max(np.abs(q_new - prev_angles)))
            n_steps = max(1, int(np.ceil(max_diff_deg / self.max_joint_step_deg)))
            for step in range(1, n_steps + 1):
                alpha = step / n_steps
                q_interp = prev_angles + alpha * (q_new - prev_angles)
                output.append((q_interp.copy(), color))
            prev_angles = q_new

        return output

    def draw_demo_path(self, starting_angles: np.ndarray) -> None:
        for index, (joint_angles, color) in enumerate(
            self.calculate_demo_ik_path(starting_angles)
        ):
            leave_paint = (index % self.paint_stride) == 0
            self.draw_painter(joint_angles, color, leave_paint=leave_paint)
            time.sleep(self.frame_delay)


def main() -> None:
    robot = RoboRollPainter(
        drawing_enabled=True,
        samples_per_segment=12,
        max_joint_step_deg=6.0,
        frame_delay=0.005,
        paint_stride=1,
        show_all_brushes=True,
    )
    starting_angles = np.zeros(4, dtype=float)

    print("RoboRoll painting demo")
    print("This follows a Lab-3-style brush path using IK and leaves paint points behind.")
    robot.draw_demo_path(starting_angles)

    final_frame = robot.ee_frame
    print("Final ee xyz =", np.array2string(final_frame[:3, 3], precision=2, suppress_small=True))
    print("Demo complete. Close the matplotlib window when finished.")
    plt.show()


if __name__ == "__main__":
    main()
