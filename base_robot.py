import math
import os
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np
import yaml


@dataclass
class JointSpec:
    name: str
    low_limit: float
    high_limit: float
    theta_offset: float = 0.0

    def is_inside_joint_limit(self, angle: float) -> bool:
        return self.low_limit <= angle <= self.high_limit


def _load_config(path: str | None = None) -> dict:
    if path is None:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_config.yaml")
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _deg_to_rad(angle: float) -> float:
    return math.radians(float(angle))


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


class BaseCustomRobot(object):
    """4-DOF custom painting robot used by the project autograder."""

    TOOL_TILT_X = -math.pi / 2

    def __init__(self, config_path: str | None = None, drawing_enabled: bool = False):
        del drawing_enabled
        cfg = _load_config(config_path)

        joint_cfgs = cfg["joints"]
        self.NUM_JOINTS = int(cfg["num_dof"])
        self.A = [float(j["a"]) for j in joint_cfgs]
        self.D = [float(j["d"]) for j in joint_cfgs]
        self.THETA_OFFSET = [_deg_to_rad(j.get("theta_offset", 0.0)) for j in joint_cfgs]

        self._joints = [
            JointSpec(
                name=str(j.get("name", f"joint_{index + 1}")),
                low_limit=_deg_to_rad(j["limits"][0]),
                high_limit=_deg_to_rad(j["limits"][1]),
                theta_offset=_deg_to_rad(j.get("theta_offset", 0.0)),
            )
            for index, j in enumerate(joint_cfgs)
        ]

        self._joint_angles = np.zeros(self.NUM_JOINTS, dtype=float)
        self._ee_frame = np.eye(4)
        self.calculate_fk(self._joint_angles.copy())

    @property
    def joints(self) -> List[JointSpec]:
        return self._joints

    @property
    def ee_frame(self) -> np.ndarray:
        return self._ee_frame.copy()

    def _check_joint_angles(self, joint_angles: np.ndarray) -> np.ndarray:
        angles = np.asarray(joint_angles, dtype=float).reshape(-1)
        if angles.shape != (self.NUM_JOINTS,):
            raise ValueError(f"Expected {self.NUM_JOINTS} joint angles, got shape {angles.shape}")

        for index, (joint, angle) in enumerate(zip(self.joints, angles), start=1):
            if not joint.is_inside_joint_limit(float(angle)):
                raise ValueError(
                    f"Joint {index} angle {math.degrees(float(angle)):.2f} deg is outside limits "
                    f"[{math.degrees(joint.low_limit):.1f}, {math.degrees(joint.high_limit):.1f}] deg"
                )
        return angles

    def _forward_position(self, joint_angles: np.ndarray) -> np.ndarray:
        return self._forward_frame(joint_angles)[:3, 3]

    def _forward_rotation(self, joint_angles: np.ndarray) -> np.ndarray:
        return self._forward_frame(joint_angles)[:3, :3]

    def _forward_frame(self, joint_angles: np.ndarray) -> np.ndarray:
        t1, t2, t3, t4 = joint_angles

        return (
            _tz(self.D[0])
            @ _homogeneous(_rot_z(t1), np.zeros(3))
            @ _homogeneous(_rot_x(self.TOOL_TILT_X), np.zeros(3))
            @ _homogeneous(_rot_z(t2), np.zeros(3))
            @ _tx(self.A[2])
            @ _homogeneous(_rot_z(t3), np.zeros(3))
            @ _tx(self.A[3])
            @ _homogeneous(_rot_z(t4), np.zeros(3))
        )

    def calculate_fk(self, joint_angles: np.ndarray) -> np.ndarray:
        angles = self._check_joint_angles(joint_angles)

        frame = self._forward_frame(angles)

        self._joint_angles = angles.copy()
        self._ee_frame = frame
        return frame.copy()

    def calculate_ik(
        self,
        ee_frame: np.ndarray,
        prev_joint_angles: np.ndarray,
    ) -> Tuple[bool, np.ndarray]:
        target = np.asarray(ee_frame, dtype=float)
        prev = np.asarray(prev_joint_angles, dtype=float).reshape(-1)

        if target.shape != (4, 4):
            return False, np.zeros(self.NUM_JOINTS)
        if prev.shape != (self.NUM_JOINTS,):
            return False, np.zeros(self.NUM_JOINTS)

        base_height = self.D[0]
        link_1 = self.A[2]
        link_2 = self.A[3]
        solutions = []
        theta1 = math.atan2(-target[0, 2], target[1, 2])
        theta1 = (theta1 + math.pi) % (2.0 * math.pi) - math.pi
        total_wrist_yaw = math.atan2(-target[2, 0], -target[2, 1])
        total_wrist_yaw = (total_wrist_yaw + math.pi) % (2.0 * math.pi) - math.pi

        for theta1_candidate in (theta1, ((theta1 + math.pi) % (2.0 * math.pi)) - math.pi):
            if not self.joints[0].is_inside_joint_limit(theta1_candidate):
                continue

            base_to_target = target[:3, 3] - np.array([0.0, 0.0, base_height])
            planar = _rot_x(-self.TOOL_TILT_X) @ (_rot_z(-theta1_candidate) @ base_to_target)
            px, py = float(planar[0]), float(planar[1])

            cos_theta3 = (px * px + py * py - link_1 * link_1 - link_2 * link_2) / (2.0 * link_1 * link_2)
            if abs(cos_theta3) > 1.0 + 1e-9:
                continue
            cos_theta3 = float(np.clip(cos_theta3, -1.0, 1.0))

            for sign in (1.0, -1.0):
                theta3 = sign * math.acos(cos_theta3)
                if not self.joints[2].is_inside_joint_limit(theta3):
                    continue

                theta2 = math.atan2(py, px) - math.atan2(
                    link_2 * math.sin(theta3),
                    link_1 + link_2 * math.cos(theta3),
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
            return False, np.zeros(self.NUM_JOINTS)

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
