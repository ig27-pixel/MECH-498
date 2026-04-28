# base_robot.py
# Isaiah Gonzalez — RoboRoll Coatings Project
# MECH 498 — Introduction to Robotics
#
# 4-DOF robotic painting arm (Modified DH convention)
#
# DH Table (Modified DH):
#   i  | alpha_{i-1} | a_{i-1} (mm) | d_i (mm) | theta_i
#   ---+-------------+--------------+----------+--------
#   1  |      0      |      0       |   500    | theta_1  (base yaw)
#   2  |   -pi/2     |      0       |    0     | theta_2  (shoulder pitch)
#   3  |      0      |    700       |    0     | theta_3  (elbow pitch)
#   4  |      0      |    500       |    0     | theta_4  (wrist pitch)

import numpy as np
import math
from typing import Tuple, List


def _dh_transform(alpha: float, a: float, d: float, theta: float) -> np.ndarray:
    """Modified DH transformation matrix."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct,    -st,    0,      a     ],
        [st*ca,  ct*ca, -sa,   -sa*d  ],
        [st*sa,  ct*sa,  ca,    ca*d  ],
        [0,      0,      0,     1     ]
    ])


class BaseCustomRobot(object):
    """4-DOF construction painting robot for RoboRoll Coatings."""

    # ── DH parameters ──────────────────────────────────────────────────────────
    ALPHA = [0.0, -math.pi / 2, 0.0, 0.0]  # alpha_{i-1}
    A     = [0.0, 0.0, 700.0, 500.0]        # a_{i-1} (mm)
    D     = [500.0, 0.0, 0.0, 0.0]          # d_i (mm)

    # ── Joint limits [low, high] in radians ────────────────────────────────────
    JOINT_LIMITS = [
        (-math.pi,       math.pi),        # J1: base yaw     ±180°
        (-math.pi / 2,   math.pi / 2),    # J2: shoulder     ±90°
        (-math.pi / 3,   2*math.pi / 3),  # J3: elbow        -60° to +120°
        (-math.pi / 2,   math.pi / 2),    # J4: wrist pitch  ±90°
    ]

    NUM_JOINTS = 4

    def __init__(self):
        # Current joint angles (radians)
        self._theta = [0.0] * self.NUM_JOINTS

    # ── Helpers ────────────────────────────────────────────────────────────────

    def _in_limit(self, i: int, theta: float) -> bool:
        lo, hi = self.JOINT_LIMITS[i]
        return lo <= theta <= hi

    def _fk_transforms(self, joint_angles) -> List[np.ndarray]:
        """Return list of individual DH transforms for each joint."""
        return [
            _dh_transform(self.ALPHA[i], self.A[i], self.D[i], joint_angles[i])
            for i in range(self.NUM_JOINTS)
        ]

    # ── FK ─────────────────────────────────────────────────────────────────────

    def calculate_fk(self, joint_angles: np.ndarray) -> np.ndarray:
        """Compute forward kinematics.

        Args:
            joint_angles: (4,) array of joint angles in radians

        Returns:
            (4,4) homogeneous transformation matrix of the end-effector (wrist)

        Raises:
            ValueError: if any joint angle is outside its limit
        """
        for i, theta in enumerate(joint_angles):
            if not self._in_limit(i, float(theta)):
                lo, hi = self.JOINT_LIMITS[i]
                raise ValueError(
                    f"Joint {i+1} angle {math.degrees(float(theta)):.2f} deg is outside "
                    f"limits [{math.degrees(lo):.1f}, {math.degrees(hi):.1f}] deg"
                )

        T = np.eye(4)
        for Ti in self._fk_transforms(joint_angles):
            T = T @ Ti
        return T

    # ── IK ─────────────────────────────────────────────────────────────────────

    def calculate_ik(self, ee_frame: np.ndarray,
                     prev_joint_angles: np.ndarray) -> Tuple[bool, np.ndarray]:
        """Compute inverse kinematics analytically.

        Closed-form solution derived from Modified DH FK:
          theta_1  — rotation column 2:  R[:,2] = [-sin(t1), cos(t1), 0]
          theta_2,3 — 2-link planar IK in the vertical plane at angle theta_1
          theta_4  — rotation row 2:     R[2,:] = [-sin(t234), -cos(t234), 0]

        Args:
            ee_frame: (4,4) target end-effector transformation matrix
            prev_joint_angles: (4,) previous joint angles for solution selection

        Returns:
            (success, joint_angles): True/False and (4,) array (zeros on failure)
        """
        px, py, pz = ee_frame[0, 3], ee_frame[1, 3], ee_frame[2, 3]
        R = ee_frame[:3, :3]

        L1 = self.A[2]   # 700 mm — upper arm
        L2 = self.A[3]   # 500 mm — forearm
        D1 = self.D[0]   # 500 mm — base height

        # ── Step 1: theta_1 from R[:,2] = [-sin(t1), cos(t1), 0] ──────────────
        s1, c1 = -R[0, 2], R[1, 2]
        if abs(s1) < 1e-9 and abs(c1) < 1e-9:
            rho = math.sqrt(px**2 + py**2)
            theta1_base = prev_joint_angles[0] if rho < 1e-6 else math.atan2(py, px)
        else:
            theta1_base = math.atan2(s1, c1)

        # Consider flipped base (arm pointing backward)
        alt = theta1_base + math.pi
        if alt > math.pi:
            alt -= 2.0 * math.pi
        theta1_candidates = [theta1_base, alt]

        # ── Steps 2–4: planar IK for each theta_1 candidate ───────────────────
        solutions = []

        for t1 in theta1_candidates:
            if not self._in_limit(0, t1):
                continue

            r = px * math.cos(t1) + py * math.sin(t1)  # signed horizontal reach
            h = D1 - pz                                  # L1*sin(t2)+L2*sin(t2+t3)

            # Law of cosines for theta_3
            cos_t3 = (r**2 + h**2 - L1**2 - L2**2) / (2.0 * L1 * L2)
            if abs(cos_t3) > 1.0 + 1e-9:
                continue
            cos_t3 = float(np.clip(cos_t3, -1.0, 1.0))

            for sign in [1, -1]:   # elbow-up and elbow-down
                t3 = sign * math.acos(cos_t3)
                if not self._in_limit(2, t3):
                    continue

                beta  = math.atan2(h, r)
                gamma = math.atan2(L2 * math.sin(t3), L1 + L2 * math.cos(t3))
                t2 = beta - gamma
                if not self._in_limit(1, t2):
                    continue

                # theta_4 from R[2,:] = [-sin(t234), -cos(t234), 0]
                t234 = math.atan2(-R[2, 0], -R[2, 1])
                t4 = (t234 - t2 - t3 + math.pi) % (2.0 * math.pi) - math.pi
                if not self._in_limit(3, t4):
                    continue

                solutions.append(np.array([t1, t2, t3, t4]))

        if not solutions:
            return False, np.zeros(self.NUM_JOINTS)

        # Select solution closest to previous joint angles
        best = min(solutions, key=lambda s: np.linalg.norm(s - prev_joint_angles))

        # FK verification (1 mm tolerance)
        try:
            T_check = self.calculate_fk(best)
        except ValueError:
            return False, np.zeros(self.NUM_JOINTS)

        if np.linalg.norm(ee_frame[:3, 3] - T_check[:3, 3]) > 1.0:
            return False, np.zeros(self.NUM_JOINTS)

        return True, best
