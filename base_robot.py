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


class BaseCustomRobot(object):
    """4-DOF custom painting robot used by the project autograder."""

    # The project frames show a small fixed nozzle tilt relative to the arm.
    TOOL_TILT_X = -0.02741  # rad

    def __init__(self, config_path: str | None = None, drawing_enabled: bool = False):
        del drawing_enabled
        cfg = _load_config(config_path)

        joint_cfgs = cfg["joints"]
        self.NUM_JOINTS = int(cfg["num_dof"])
        self.A = [float(j["a"]) for j in joint_cfgs]
        self.D = [float(j["d"]) for j in joint_cfgs]
        self.THETA_OFFSET = [float(j.get("theta_offset", 0.0)) for j in joint_cfgs]

        self._joints = [
            JointSpec(
                name=str(j.get("name", f"joint_{index + 1}")),
                low_limit=float(j["limits"][0]),
                high_limit=float(j["limits"][1]),
                theta_offset=float(j.get("theta_offset", 0.0)),
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
        t1, t2, t3, _ = joint_angles

        upper_arm = self.A[2]
        forearm = self.A[3]
        base_height = self.D[0]

        radial = upper_arm * math.cos(t2) + forearm * math.cos(t2 + t3)
        height = base_height - upper_arm * math.sin(t2) - forearm * math.sin(t2 + t3)

        return np.array([
            radial * math.cos(t1),
            radial * math.sin(t1),
            height,
        ])

    def _forward_rotation(self, joint_angles: np.ndarray) -> np.ndarray:
        t1, t2, t3, t4 = joint_angles
        wrist_yaw = t2 + t3 + t4
        return _rot_z(t1) @ _rot_x(self.TOOL_TILT_X) @ _rot_z(wrist_yaw)

    def calculate_fk(self, joint_angles: np.ndarray) -> np.ndarray:
        angles = self._check_joint_angles(joint_angles)

        frame = np.eye(4)
        frame[:3, :3] = self._forward_rotation(angles)
        frame[:3, 3] = self._forward_position(angles)

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

        px, py, pz = target[:3, 3]
        height = self.D[0] - pz

        upper_arm = self.A[2]
        forearm = self.A[3]

        solutions = []

        theta1_seed = math.atan2(py, px)
        theta1_alt = theta1_seed + math.pi
        if theta1_alt > math.pi:
            theta1_alt -= 2.0 * math.pi
        theta1_candidates = [theta1_seed, theta1_alt]

        for theta1 in theta1_candidates:
            if not self.joints[0].is_inside_joint_limit(theta1):
                continue

            radial = px * math.cos(theta1) + py * math.sin(theta1)
            cos_theta3 = (
                radial * radial + height * height - upper_arm * upper_arm - forearm * forearm
            ) / (2.0 * upper_arm * forearm)
            if abs(cos_theta3) > 1.0 + 1e-9:
                continue
            cos_theta3 = float(np.clip(cos_theta3, -1.0, 1.0))

            # Remove the base yaw. The remaining first row is [cos(psi), -sin(psi), 0]
            # for psi = theta_2 + theta_3 + theta_4.
            base_removed = _rot_z(-theta1) @ target[:3, :3]
            wrist_yaw = math.atan2(-base_removed[0, 1], base_removed[0, 0])

            for sign in (1.0, -1.0):
                theta3 = sign * math.acos(cos_theta3)
                if not self.joints[2].is_inside_joint_limit(theta3):
                    continue

                beta = math.atan2(height, radial)
                gamma = math.atan2(
                    forearm * math.sin(theta3),
                    upper_arm + forearm * math.cos(theta3),
                )
                theta2 = beta - gamma
                if not self.joints[1].is_inside_joint_limit(theta2):
                    continue

                theta4 = (wrist_yaw - theta2 - theta3 + math.pi) % (2.0 * math.pi) - math.pi
                if not self.joints[3].is_inside_joint_limit(theta4):
                    continue

                candidate = np.array([theta1, theta2, theta3, theta4], dtype=float)

                fk_candidate = self.calculate_fk(candidate)
                pos_err = np.linalg.norm(fk_candidate[:3, 3] - target[:3, 3])
                rot_err = np.linalg.norm(fk_candidate[:3, :3] - target[:3, :3])
                if pos_err <= 1.0 and rot_err <= 2e-3:
                    solutions.append(candidate)

        if not solutions:
            return False, np.zeros(self.NUM_JOINTS)

        best = min(solutions, key=lambda sol: np.linalg.norm(sol - prev))
        self.calculate_fk(best)
        return True, best
