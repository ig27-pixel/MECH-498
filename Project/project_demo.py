import time
from typing import Dict, List, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np

from RoboRoll import RoboRoll
from robot_components import Brush


class RoboRollPainter(RoboRoll):
    """Project demo that paints a compact smiley on a reachable wall plane."""

    def __init__(
        self,
        drawing_enabled: bool = True,
        wall_x: float = 1100.0,
        wall_center_y: float = 0.0,
        wall_center_z: float = 710.0,
        frame_delay: float = 0.004,
        paint_stride: int = 1,
        show_all_brushes: bool = True,
        stroke_interp_steps: int = 6,
        travel_interp_steps: int = 10,
        bank_size_per_color: int = 400,
        full_robot_view: bool = True,
    ):
        super().__init__(drawing_enabled=drawing_enabled)
        self.brush = Brush(self.ax, drawing_enabled=drawing_enabled)
        self.wall_x = float(wall_x)
        self.wall_center_y = float(wall_center_y)
        self.wall_center_z = float(wall_center_z)
        self.frame_delay = max(0.0, float(frame_delay))
        self.paint_stride = max(1, int(paint_stride))
        self.show_all_brushes = bool(show_all_brushes)
        self.stroke_interp_steps = max(2, int(stroke_interp_steps))
        self.travel_interp_steps = max(2, int(travel_interp_steps))
        self.bank_size_per_color = max(400, int(bank_size_per_color))
        self.full_robot_view = bool(full_robot_view)
        self._bank_rng = np.random.default_rng(498)
        self._reachable_banks: Dict[int, List[Tuple[np.ndarray, np.ndarray]]] = {}
        self._ee_marker = None

        if self._drawing_enabled:
            if not self.full_robot_view:
                self._configure_demo_view()
            self._draw_wall()

    def _configure_demo_view(self) -> None:
        if not self._drawing_enabled or self.ax is None:
            return

        self.ax.set_xlim([850.0, 1450.0])
        self.ax.set_ylim([-260.0, 260.0])
        self.ax.set_zlim([640.0, 790.0])
        self.ax.view_init(elev=18, azim=58)
        try:
            self.ax.set_box_aspect((600.0, 520.0, 150.0))
        except Exception:
            pass

    def _draw_wall(self) -> None:
        if not self._drawing_enabled or self.ax is None:
            return

        y = np.linspace(self.wall_center_y - 140.0, self.wall_center_y + 140.0, 2)
        z = np.linspace(self.wall_center_z - 45.0, self.wall_center_z + 45.0, 2)
        yy, zz = np.meshgrid(y, z)
        xx = np.full_like(yy, self.wall_x)
        self.ax.plot_surface(xx, yy, zz, color="#d9d9d9", alpha=0.30, shade=False)
        self.ax.plot(
            [self.wall_x, self.wall_x, self.wall_x, self.wall_x, self.wall_x],
            [
                self.wall_center_y - 140.0,
                self.wall_center_y + 140.0,
                self.wall_center_y + 140.0,
                self.wall_center_y - 140.0,
                self.wall_center_y - 140.0,
            ],
            [
                self.wall_center_z - 45.0,
                self.wall_center_z - 45.0,
                self.wall_center_z + 45.0,
                self.wall_center_z + 45.0,
                self.wall_center_z - 45.0,
            ],
            color="#8f8f8f",
            linewidth=1.3,
            alpha=0.8,
        )

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
        self._draw_end_effector_marker()
        plt.pause(0.00001)

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
                markersize=11,
                color="black",
                markeredgecolor="white",
                markeredgewidth=1.2,
                linestyle="None",
                zorder=10,
            )
        else:
            self._ee_marker.set_xdata([position[0]])
            self._ee_marker.set_ydata([position[1]])
            self._ee_marker.set_3d_properties([position[2]])

    def _sample_reachable_bank(self, color: int) -> List[Tuple[np.ndarray, np.ndarray]]:
        if color in self._reachable_banks:
            return self._reachable_banks[color]

        bank: List[Tuple[np.ndarray, np.ndarray]] = []
        self.brush.selection = int(color)

        while len(bank) < self.bank_size_per_color:
            q = np.array(
                [
                    self._bank_rng.uniform(-0.4, 0.4),
                    self._bank_rng.uniform(-0.4, 0.4),
                    self._bank_rng.uniform(-0.2, 0.8),
                    self._bank_rng.uniform(-0.4, 0.4),
                ],
                dtype=float,
            )
            ee_frame = self.calculate_fk(q)
            brush_frame = ee_frame @ self.brush.selected_brush_frame_dh
            position = brush_frame[:3, 3].copy()

            # Keep samples in the band where the smiley wall lives.
            if (
                abs(position[0] - self.wall_x) <= 220.0
                and abs(position[1] - self.wall_center_y) <= 180.0
                and abs(position[2] - self.wall_center_z) <= 35.0
            ):
                bank.append((q.copy(), position))

        self._reachable_banks[color] = bank
        return bank

    def _build_smiley_strokes(self) -> List[Tuple[int, List[np.ndarray]]]:
        center = np.array([self.wall_x, self.wall_center_y, self.wall_center_z], dtype=float)
        strokes: List[Tuple[int, List[np.ndarray]]] = []

        outline_points = []
        for theta in np.linspace(0.0, 2.0 * np.pi, 32, endpoint=False):
            outline_points.append(
                center
                + np.array(
                    [
                        0.0,
                        88.0 * np.cos(theta),
                        22.0 * np.sin(theta),
                    ],
                    dtype=float,
                )
            )
        strokes.append((2, outline_points))  # yellow outline

        left_eye_center = center + np.array([0.0, -34.0, 8.0], dtype=float)
        left_eye = []
        for theta in np.linspace(0.0, 2.0 * np.pi, 10, endpoint=False):
            left_eye.append(
                left_eye_center
                + np.array([0.0, 11.0 * np.cos(theta), 4.0 * np.sin(theta)], dtype=float)
            )
        strokes.append((4, left_eye))  # blue left eye

        right_eye_center = center + np.array([0.0, 34.0, 8.0], dtype=float)
        right_eye = []
        for theta in np.linspace(0.0, 2.0 * np.pi, 10, endpoint=False):
            right_eye.append(
                right_eye_center
                + np.array([0.0, 11.0 * np.cos(theta), 4.0 * np.sin(theta)], dtype=float)
            )
        strokes.append((3, right_eye))  # orange right eye

        smile_points = []
        for theta in np.linspace(0.16 * np.pi, 0.84 * np.pi, 20):
            smile_points.append(
                center
                + np.array(
                    [
                        0.0,
                        48.0 * np.cos(theta),
                        -10.0 - 13.0 * np.sin(theta),
                    ],
                    dtype=float,
                )
            )
        strokes.append((1, smile_points))  # purple smile

        return strokes

    def _select_joint_for_target(
        self,
        color: int,
        target_position: np.ndarray,
        prev_angles: np.ndarray,
    ) -> np.ndarray:
        bank = self._sample_reachable_bank(color)
        best_angles = bank[0][0]
        best_cost = float("inf")

        for candidate_angles, candidate_position in bank:
            position_cost = (
                4.0 * abs(candidate_position[0] - target_position[0])
                + np.linalg.norm(candidate_position[1:] - target_position[1:])
            )
            smoothness_cost = 18.0 * np.linalg.norm(candidate_angles - prev_angles)
            total_cost = position_cost + smoothness_cost
            if total_cost < best_cost:
                best_cost = total_cost
                best_angles = candidate_angles

        return best_angles.copy()

    def _interpolate_segment(
        self,
        start: np.ndarray,
        end: np.ndarray,
        color: int,
        paint: bool,
        steps: int,
    ) -> List[Tuple[np.ndarray, int, bool]]:
        frames: List[Tuple[np.ndarray, int, bool]] = []
        for alpha in np.linspace(0.0, 1.0, steps, endpoint=False)[1:]:
            blend = alpha * alpha * (3.0 - 2.0 * alpha)
            frames.append((start + blend * (end - start), color, paint))
        frames.append((end.copy(), color, paint))
        return frames

    def build_smiley_joint_path(
        self,
        starting_angles: np.ndarray,
    ) -> List[Tuple[np.ndarray, int, bool]]:
        path: List[Tuple[np.ndarray, int, bool]] = []
        prev_angles = np.asarray(starting_angles, dtype=float).reshape(-1)

        for stroke_index, (color, target_positions) in enumerate(self._build_smiley_strokes()):
            stroke_waypoints: List[np.ndarray] = []
            selector_prev = prev_angles.copy()
            for target_position in target_positions:
                next_angles = self._select_joint_for_target(color, target_position, selector_prev)
                stroke_waypoints.append(next_angles)
                selector_prev = next_angles

            if not stroke_waypoints:
                continue

            # Move to the start of the next stroke without painting.
            path.extend(
                self._interpolate_segment(
                    prev_angles,
                    stroke_waypoints[0],
                    0,
                    False,
                    self.travel_interp_steps,
                )
            )
            prev_angles = stroke_waypoints[0]

            # Paint through the stroke.
            for next_angles in stroke_waypoints[1:]:
                path.extend(
                    self._interpolate_segment(
                        prev_angles,
                        next_angles,
                        color,
                        True,
                        self.stroke_interp_steps,
                    )
                )
                prev_angles = next_angles

            # Slight pause at the end of each stroke.
            if stroke_index < len(self._build_smiley_strokes()) - 1:
                path.append((prev_angles.copy(), 0, False))

        # Return home cleanly without painting.
        home = np.zeros(4, dtype=float)
        path.extend(
            self._interpolate_segment(
                prev_angles,
                home,
                0,
                False,
                self.travel_interp_steps,
            )
        )
        return path

    def draw_demo_path(self, starting_angles: np.ndarray) -> None:
        for index, (joint_angles, color, paint_now) in enumerate(
            self.build_smiley_joint_path(starting_angles)
        ):
            leave_paint = paint_now and (index % self.paint_stride) == 0
            self.draw_painter(joint_angles, color, leave_paint=leave_paint)
            time.sleep(self.frame_delay)


def main() -> None:
    robot = RoboRollPainter(
        drawing_enabled=True,
        wall_x=1100.0,
        wall_center_y=0.0,
        wall_center_z=710.0,
        frame_delay=0.004,
        paint_stride=1,
        show_all_brushes=True,
        stroke_interp_steps=6,
        travel_interp_steps=10,
        bank_size_per_color=400,
        full_robot_view=True,
    )
    starting_angles = np.zeros(4, dtype=float)

    print("RoboRoll smiley-wall demo")
    print("Painting a compact smiley on a reachable wall near x = 1100 mm.")
    robot.draw_demo_path(starting_angles)

    final_frame = robot.ee_frame
    print("Final ee xyz =", np.array2string(final_frame[:3, 3], precision=2, suppress_small=True))
    print("Demo complete. Close the matplotlib window when finished.")
    plt.show()


if __name__ == "__main__":
    main()
