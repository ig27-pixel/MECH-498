import math
import os
import sys
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "Lab-2"))
import general_utility as general
from robot_components import Joint, Link


def _rot_x(theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([
        [1.0, 0.0, 0.0],
        [0.0, c, -s],
        [0.0, s, c],
    ])


def _rot_z(theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ])


def _homogeneous(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    frame = np.eye(4)
    frame[:3, :3] = rotation
    frame[:3, 3] = translation
    return frame


def _tx(distance: float) -> np.ndarray:
    return _homogeneous(np.eye(3), np.array([distance, 0.0, 0.0]))


def _tz(distance: float) -> np.ndarray:
    return _homogeneous(np.eye(3), np.array([0.0, 0.0, distance]))


class RoboRoll(object):
    """4-DOF construction painting robot for RoboRoll Coatings."""

    A_2 = 700.0
    A_3 = 500.0
    D_1 = 500.0
    NOZZLE_OFFSET = 200.0
    TOOL_TILT_X = -math.pi / 2

    J1_LIMITS = (-math.pi, math.pi)
    J2_LIMITS = (-math.pi / 2, math.pi / 2)
    J3_LIMITS = (-math.pi / 3, 2.0 * math.pi / 3)
    J4_LIMITS = (-math.pi / 2, math.pi / 2)

    def __init__(self, drawing_enabled: bool = True):
        self._drawing_enabled = drawing_enabled
        self._drawn_once = False
        self._joint_angles = np.zeros(4, dtype=float)

        self.colors = [
            [0.10, 0.10, 0.80],
            [0.10, 0.65, 0.20],
            [0.85, 0.35, 0.00],
            [0.65, 0.00, 0.70],
        ]

        ws = self.A_2 + self.A_3 + self.NOZZLE_OFFSET + 200.0
        if self._drawing_enabled:
            self._create_plot(ws)
        else:
            self.fig = None
            self.ax = None

        frame_size = 65.0
        self._joint_1 = Joint(self.ax, self.colors[0], self._drawing_enabled, frame_size=frame_size)
        self._joint_2 = Joint(self.ax, self.colors[1], self._drawing_enabled, frame_size=frame_size)
        self._joint_3 = Joint(self.ax, self.colors[2], self._drawing_enabled, frame_size=frame_size)
        self._joint_4 = Joint(self.ax, self.colors[3], self._drawing_enabled, frame_size=frame_size)

        self._joint_1.set_joint_limits(*self.J1_LIMITS)
        self._joint_2.set_joint_limits(*self.J2_LIMITS)
        self._joint_3.set_joint_limits(*self.J3_LIMITS)
        self._joint_4.set_joint_limits(*self.J4_LIMITS)

        self._links = [Link(self.ax, self.colors[i], drawing_enabled) for i in range(4)]
        self._update_joint_transforms(self._joint_angles)

    @property
    def joints(self) -> List[Joint]:
        return [self._joint_1, self._joint_2, self._joint_3, self._joint_4]

    @property
    def ee_frame(self) -> np.ndarray:
        return self._forward_frame(self._joint_angles)

    @property
    def nozzle_frame(self) -> np.ndarray:
        t_nozzle = np.eye(4)
        t_nozzle[0, 3] = self.NOZZLE_OFFSET
        return self.ee_frame @ t_nozzle

    def _forward_frame(self, joint_angles: np.ndarray) -> np.ndarray:
        t1, t2, t3, t4 = joint_angles
        return (
            _tz(self.D_1)
            @ _homogeneous(_rot_z(t1), np.zeros(3))
            @ _homogeneous(_rot_x(self.TOOL_TILT_X), np.zeros(3))
            @ _homogeneous(_rot_z(t2), np.zeros(3))
            @ _tx(self.A_2)
            @ _homogeneous(_rot_z(t3), np.zeros(3))
            @ _tx(self.A_3)
            @ _homogeneous(_rot_z(t4), np.zeros(3))
        )

    def _update_joint_transforms(self, joint_angles: np.ndarray) -> None:
        t1, t2, t3, t4 = joint_angles
        self._joint_1.set_dh_transform(_tz(self.D_1) @ _homogeneous(_rot_z(t1), np.zeros(3)))
        self._joint_2.set_dh_transform(
            _homogeneous(_rot_x(self.TOOL_TILT_X), np.zeros(3))
            @ _homogeneous(_rot_z(t2), np.zeros(3))
        )
        self._joint_3.set_dh_transform(_tx(self.A_2) @ _homogeneous(_rot_z(t3), np.zeros(3)))
        self._joint_4.set_dh_transform(_tx(self.A_3) @ _homogeneous(_rot_z(t4), np.zeros(3)))

    def calculate_fk(self, joint_angles: np.ndarray) -> np.ndarray:
        general.check_proper_numpy_format(joint_angles, (4,))
        angles = np.asarray(joint_angles, dtype=float).reshape(-1)

        for index, (joint, angle) in enumerate(zip(self.joints, angles), start=1):
            if not joint.is_inside_joint_limit(float(angle)):
                raise ValueError(
                    f"Joint {index} angle {math.degrees(float(angle)):.2f} deg is outside limits "
                    f"[{math.degrees(joint.low_limit):.1f}, {math.degrees(joint.high_limit):.1f}] deg"
                )

        self._joint_angles = angles.copy()
        self._update_joint_transforms(self._joint_angles)
        return self.ee_frame

    def calculate_ik(
        self,
        ee_frame: np.ndarray,
        prev_joint_angles: np.ndarray,
    ) -> Tuple[bool, np.ndarray]:
        target = np.asarray(ee_frame, dtype=float)
        prev = np.asarray(prev_joint_angles, dtype=float).reshape(-1)
        if target.shape != (4, 4) or prev.shape != (4,):
            return False, np.zeros(4)

        solutions = []
        theta1 = math.atan2(-target[0, 2], target[1, 2])
        theta1 = (theta1 + math.pi) % (2.0 * math.pi) - math.pi
        total_wrist_yaw = math.atan2(-target[2, 0], -target[2, 1])
        total_wrist_yaw = (total_wrist_yaw + math.pi) % (2.0 * math.pi) - math.pi

        for theta1_candidate in (theta1, ((theta1 + math.pi) % (2.0 * math.pi)) - math.pi):
            if not self.joints[0].is_inside_joint_limit(theta1_candidate):
                continue

            base_to_target = target[:3, 3] - np.array([0.0, 0.0, self.D_1])
            planar = _rot_x(-self.TOOL_TILT_X) @ (_rot_z(-theta1_candidate) @ base_to_target)
            px, py = float(planar[0]), float(planar[1])

            cos_theta3 = (
                px * px + py * py - self.A_2 * self.A_2 - self.A_3 * self.A_3
            ) / (2.0 * self.A_2 * self.A_3)
            if abs(cos_theta3) > 1.0 + 1e-9:
                continue
            cos_theta3 = float(np.clip(cos_theta3, -1.0, 1.0))

            for sign in (1.0, -1.0):
                theta3 = sign * math.acos(cos_theta3)
                if not self.joints[2].is_inside_joint_limit(theta3):
                    continue

                theta2 = math.atan2(py, px) - math.atan2(
                    self.A_3 * math.sin(theta3),
                    self.A_2 + self.A_3 * math.cos(theta3),
                )
                theta2 = (theta2 + math.pi) % (2.0 * math.pi) - math.pi
                if not self.joints[1].is_inside_joint_limit(theta2):
                    continue

                theta4 = total_wrist_yaw - theta2 - theta3
                theta4 = (theta4 + math.pi) % (2.0 * math.pi) - math.pi
                if not self.joints[3].is_inside_joint_limit(theta4):
                    continue

                candidate = np.array([theta1_candidate, theta2, theta3, theta4], dtype=float)
                fk_candidate = self._forward_frame(candidate)
                pos_err = np.linalg.norm(fk_candidate[:3, 3] - target[:3, 3])
                rot_err = np.linalg.norm(fk_candidate[:3, :3] - target[:3, :3])
                if pos_err <= 1e-3 and rot_err <= 1e-5:
                    solutions.append(candidate)

        if not solutions:
            return False, np.zeros(4)

        distances = [np.linalg.norm(sol - prev) for sol in solutions]
        min_distance = min(distances)
        close_solutions = [
            sol for sol, distance in zip(solutions, distances)
            if distance <= min_distance + 1e-4
        ]
        singular_close = [sol for sol in close_solutions if abs(sol[2]) <= 1e-4]
        if singular_close:
            best = max(singular_close, key=lambda sol: sol[2])
        else:
            best = min(solutions, key=lambda sol: np.linalg.norm(sol - prev))

        self.calculate_fk(best)
        return True, best

    def draw_robot(self, joint_angles: np.ndarray) -> None:
        general.check_proper_numpy_format(joint_angles, (4,))
        self.calculate_fk(joint_angles)

        if not self._drawn_once:
            plt.show(block=False)
            plt.pause(0.5)
            self._drawn_once = True

        base_frame = np.eye(4)
        current_transform = base_frame.copy()
        for index, joint in enumerate(self.joints):
            prev_transform = current_transform.copy()
            current_transform = current_transform @ joint.dh_transform
            joint.set_final_transform(current_transform)
            joint.draw()
            self._links[index].update_frames(prev_transform, current_transform)
            self._links[index].draw()

        plt.pause(0.0001)

    def _create_plot(self, ws: float):
        self.fig = plt.figure(figsize=(9, 9), facecolor="w")
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_xlim([-ws, ws])
        self.ax.set_ylim([-ws, ws])
        self.ax.set_zlim([0, ws])
        self.ax.set_xlabel("X (mm)", fontsize=13)
        self.ax.set_ylabel("Y (mm)", fontsize=13)
        self.ax.set_zlabel("Z (mm)", fontsize=13)
        self.ax.set_title("RoboRoll Coatings - Painting Robot", fontsize=15)
        plt.grid(True)
