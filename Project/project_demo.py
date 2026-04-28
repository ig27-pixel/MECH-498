import time
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np

from RoboRoll import RoboRoll
from robot_components import Brush


class RoboRollRoomDemo(RoboRoll):
    """Fresh project demo with a full-room view and home-start motion sequence."""

    ROOM_SIZE_MM = 2000.0

    def __init__(
        self,
        drawing_enabled: bool = True,
        frame_delay: float = 0.01,
        show_all_brushes: bool = True,
        paint_stride: int = 1,
        samples_per_segment: int = 18,
    ):
        super().__init__(drawing_enabled=drawing_enabled)
        self.brush = Brush(self.ax, drawing_enabled=drawing_enabled)
        self.frame_delay = max(0.0, float(frame_delay))
        self.show_all_brushes = bool(show_all_brushes)
        self.paint_stride = max(1, int(paint_stride))
        self.samples_per_segment = max(3, int(samples_per_segment))
        self._ee_marker = None

        if self._drawing_enabled:
            self._configure_room_view()
            self._draw_room()

    def _configure_room_view(self) -> None:
        if not self._drawing_enabled or self.ax is None:
            return

        self.ax.set_xlim([0.0, self.ROOM_SIZE_MM])
        self.ax.set_ylim([-self.ROOM_SIZE_MM / 2.0, self.ROOM_SIZE_MM / 2.0])
        self.ax.set_zlim([0.0, self.ROOM_SIZE_MM])
        self.ax.set_xlabel("X (mm)", fontsize=13)
        self.ax.set_ylabel("Y (mm)", fontsize=13)
        self.ax.set_zlabel("Z (mm)", fontsize=13)
        self.ax.set_title("RoboRoll Demo Room", fontsize=15)
        self.ax.view_init(elev=18, azim=52)
        try:
            self.ax.set_box_aspect((2000.0, 2000.0, 2000.0))
        except Exception:
            pass

    def _draw_room(self) -> None:
        if not self._drawing_enabled or self.ax is None:
            return

        x0, x1 = 0.0, self.ROOM_SIZE_MM
        y0, y1 = -self.ROOM_SIZE_MM / 2.0, self.ROOM_SIZE_MM / 2.0
        z0, z1 = 0.0, self.ROOM_SIZE_MM

        corners = np.array([
            [x0, y0, z0],
            [x1, y0, z0],
            [x1, y1, z0],
            [x0, y1, z0],
            [x0, y0, z1],
            [x1, y0, z1],
            [x1, y1, z1],
            [x0, y1, z1],
        ])
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]
        for start, end in edges:
            self.ax.plot(
                [corners[start, 0], corners[end, 0]],
                [corners[start, 1], corners[end, 1]],
                [corners[start, 2], corners[end, 2]],
                color="#9a9a9a",
                linewidth=1.2,
                alpha=0.9,
            )

        wall_x = 1500.0
        wall_y = np.linspace(-260.0, 260.0, 2)
        wall_z = np.linspace(520.0, 900.0, 2)
        yy, zz = np.meshgrid(wall_y, wall_z)
        xx = np.full_like(yy, wall_x)
        self.ax.plot_surface(xx, yy, zz, color="#d9d9d9", alpha=0.18, shade=False)

    def _draw_end_effector_marker(self) -> None:
        if not self._drawing_enabled or self.ax is None:
            return

        position = self.ee_frame[:3, 3]
        if self._ee_marker is None:
            (self._ee_marker,) = self.ax.plot(
                [position[0]],
                [position[1]],
                [position[2]],
                marker="o",
                markersize=10,
                color="black",
                markeredgecolor="white",
                markeredgewidth=1.1,
                linestyle="None",
            )
        else:
            self._ee_marker.set_xdata([position[0]])
            self._ee_marker.set_ydata([position[1]])
            self._ee_marker.set_3d_properties([position[2]])

    def draw_demo_pose(
        self,
        joint_angles: np.ndarray,
        color: int = 0,
        leave_paint: bool = False,
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
        self._draw_end_effector_marker()
        plt.pause(0.00001)

    def _interpolate_waypoints(
        self,
        waypoints: List[Tuple[np.ndarray, int, bool]],
    ) -> List[Tuple[np.ndarray, int, bool]]:
        frames: List[Tuple[np.ndarray, int, bool]] = []
        for (start, color, paint), (end, _, _) in zip(waypoints[:-1], waypoints[1:]):
            for alpha in np.linspace(0.0, 1.0, self.samples_per_segment, endpoint=False):
                blend = alpha * alpha * (3.0 - 2.0 * alpha)
                q = start + blend * (end - start)
                frames.append((q, color, paint))
        frames.append(waypoints[-1])
        return frames

    def build_demo_path(self) -> List[Tuple[np.ndarray, int, bool]]:
        home = np.zeros(4, dtype=float)
        waypoints: List[Tuple[np.ndarray, int, bool]] = [
            (home, 0, False),
            (np.array([0.08, -0.06, 0.24, -0.12], dtype=float), 0, False),
            (np.array([0.16, -0.20, 0.48, -0.18], dtype=float), 1, True),
            (np.array([0.26, -0.08, 0.62, -0.22], dtype=float), 1, True),
            (np.array([0.10, 0.10, 0.28, -0.04], dtype=float), 2, True),
            (np.array([-0.06, 0.16, 0.18, 0.08], dtype=float), 2, True),
            (np.array([-0.18, 0.04, 0.40, 0.14], dtype=float), 3, True),
            (np.array([0.00, -0.10, 0.56, -0.08], dtype=float), 3, True),
            (np.array([0.20, 0.02, 0.34, -0.12], dtype=float), 4, True),
            (np.array([0.04, 0.14, 0.16, 0.04], dtype=float), 4, True),
            (home, 0, False),
        ]
        return self._interpolate_waypoints(waypoints)

    def run_demo(self, starting_angles: np.ndarray | None = None) -> None:
        if starting_angles is None:
            starting_angles = np.zeros(4, dtype=float)

        starting_angles = np.asarray(starting_angles, dtype=float).reshape(-1)
        self.draw_demo_pose(starting_angles, color=0, leave_paint=False)
        time.sleep(0.8)

        for index, (joint_angles, color, should_paint) in enumerate(self.build_demo_path()):
            leave_paint = should_paint and (index % self.paint_stride == 0)
            self.draw_demo_pose(joint_angles, color=color, leave_paint=leave_paint)
            time.sleep(self.frame_delay)


def main() -> None:
    robot = RoboRollRoomDemo(
        drawing_enabled=True,
        frame_delay=0.01,
        show_all_brushes=True,
        paint_stride=1,
        samples_per_segment=18,
    )
    starting_angles = np.zeros(4, dtype=float)

    print("RoboRoll room demo")
    print("Starting from home at [0, 0, 0, 0] inside a 2000 mm room box.")
    robot.run_demo(starting_angles)

    final_frame = robot.ee_frame
    print("Final ee xyz =", np.array2string(final_frame[:3, 3], precision=2, suppress_small=True))
    print("Demo complete. Close the matplotlib window when finished.")
    plt.show()


if __name__ == "__main__":
    main()
