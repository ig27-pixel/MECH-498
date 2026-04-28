# RoboRoll.py
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
#
# End-effector: fixed 200 mm offset along x_4 to spray nozzle

import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import Tuple, List
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'Lab-2'))
import general_utility as general
from robot_components import Link, Joint


class RoboRoll(object):
    """4-DOF construction painting robot for RoboRoll Coatings."""

    # ── DH parameters ──────────────────────────────────────────────────────────
    # alpha_{i-1} (rad)
    ALPHA_0 = 0.0
    ALPHA_1 = -math.pi / 2
    ALPHA_2 = 0.0
    ALPHA_3 = 0.0

    # a_{i-1} (mm) — link lengths
    A_0 = 0.0    # base to shoulder (no horizontal offset)
    A_1 = 0.0    # shoulder to elbow (no horizontal offset)
    A_2 = 700.0  # upper arm length
    A_3 = 500.0  # forearm length

    # d_i (mm) — joint offsets along z
    D_1 = 500.0  # base column height
    D_2 = 0.0
    D_3 = 0.0
    D_4 = 0.0

    # Fixed tool offset: spray nozzle is 200 mm along x_4
    NOZZLE_OFFSET = 200.0

    # ── Joint limits ───────────────────────────────────────────────────────────
    J1_LIMITS = (-math.pi,        math.pi)         # ±180° base yaw
    J2_LIMITS = (-math.pi / 2,    math.pi / 2)     # ±90°  shoulder pitch
    J3_LIMITS = (-math.pi / 3,    2 * math.pi / 3) # -60° to 120° elbow
    J4_LIMITS = (-math.pi / 2,    math.pi / 2)     # ±90°  wrist pitch

    def __init__(self, drawing_enabled: bool = True):
        self._drawing_enabled = drawing_enabled
        self._drawn_once = False

        self.colors = [
            [0.10, 0.10, 0.80],  # joint 1 — blue
            [0.10, 0.65, 0.20],  # joint 2 — green
            [0.85, 0.35, 0.00],  # joint 3 — orange
            [0.65, 0.00, 0.70],  # joint 4 — purple
        ]

        ws = self.A_2 + self.A_3 + self.NOZZLE_OFFSET + 200

        if self._drawing_enabled:
            self._create_plot(ws)
        else:
            self.fig = None
            self.ax = None

        self._joint_1: Joint = None
        self._joint_2: Joint = None
        self._joint_3: Joint = None
        self._joint_4: Joint = None

        self._setup_joints()

        self._links = [
            Link(self.ax, self.colors[i], drawing_enabled) for i in range(4)
        ]

    # ── Properties ─────────────────────────────────────────────────────────────

    @property
    def joints(self) -> List[Joint]:
        return [self._joint_1, self._joint_2, self._joint_3, self._joint_4]

    @property
    def ee_frame(self) -> np.ndarray:
        """Wrist (joint 4) end-effector frame."""
        return (self._joint_1.dh_transform @ self._joint_2.dh_transform @
                self._joint_3.dh_transform @ self._joint_4.dh_transform)

    @property
    def nozzle_frame(self) -> np.ndarray:
        """Spray nozzle frame — 200 mm offset along x_4 from the wrist."""
        T_nozzle = np.eye(4)
        T_nozzle[0, 3] = self.NOZZLE_OFFSET
        return self.ee_frame @ T_nozzle

    # ── Setup ──────────────────────────────────────────────────────────────────

    def _setup_joints(self):
        ax = self.ax

        self._joint_1 = Joint(ax, self.colors[0], self._drawing_enabled)
        self._joint_1.set_joint_limits(*self.J1_LIMITS)
        self._joint_1.set_dh_value_alpha(self.ALPHA_0)
        self._joint_1.set_dh_value_a(self.A_0)
        self._joint_1.set_dh_value_d(self.D_1)

        self._joint_2 = Joint(ax, self.colors[1], self._drawing_enabled)
        self._joint_2.set_joint_limits(*self.J2_LIMITS)
        self._joint_2.set_dh_value_alpha(self.ALPHA_1)
        self._joint_2.set_dh_value_a(self.A_1)
        self._joint_2.set_dh_value_d(self.D_2)

        self._joint_3 = Joint(ax, self.colors[2], self._drawing_enabled)
        self._joint_3.set_joint_limits(*self.J3_LIMITS)
        self._joint_3.set_dh_value_alpha(self.ALPHA_2)
        self._joint_3.set_dh_value_a(self.A_2)
        self._joint_3.set_dh_value_d(self.D_3)

        self._joint_4 = Joint(ax, self.colors[3], self._drawing_enabled)
        self._joint_4.set_joint_limits(*self.J4_LIMITS)
        self._joint_4.set_dh_value_alpha(self.ALPHA_3)
        self._joint_4.set_dh_value_a(self.A_3)
        self._joint_4.set_dh_value_d(self.D_4)

        if self._drawing_enabled:
            for joint in self.joints:
                joint.draw()

    # ── FK ─────────────────────────────────────────────────────────────────────

    def calculate_fk(self, joint_angles: np.ndarray) -> np.ndarray:
        """Compute forward kinematics and return the end-effector frame.

        Args:
            joint_angles: (4,) array — [theta1, theta2, theta3, theta4] in radians

        Returns:
            (4,4) homogeneous transformation matrix of the spray nozzle

        Raises:
            ValueError: if any joint angle is outside its limit
        """
        for i, (joint, theta) in enumerate(zip(self.joints, joint_angles), start=1):
            if not joint.is_inside_joint_limit(float(theta)):
                raise ValueError(
                    f"Joint {i} angle {math.degrees(float(theta)):.2f}° is outside limits "
                    f"[{math.degrees(joint.low_limit):.1f}°, {math.degrees(joint.high_limit):.1f}°]"
                )

        self._joint_1.set_theta(joint_angles[0])
        self._joint_2.set_theta(joint_angles[1])
        self._joint_3.set_theta(joint_angles[2])
        self._joint_4.set_theta(joint_angles[3])

        return self.ee_frame

    # ── IK ─────────────────────────────────────────────────────────────────────

    def calculate_ik(self, ee_frame: np.ndarray,
                     prev_joint_angles: np.ndarray) -> Tuple[bool, np.ndarray]:
        """Compute inverse kinematics analytically.

        Derivation (Modified DH FK gives closed-form):
          theta_1  — from rotation column 2: R[:,2] = [-sin(t1), cos(t1), 0]
          theta_2,3 — 2-link planar IK in the vertical plane at angle theta_1
          theta_4  — from rotation row 2:   R[2,:] = [-sin(t234), -cos(t234), 0]

        Args:
            ee_frame: (4,4) target end-effector (wrist) transformation matrix
            prev_joint_angles: (4,) previous joint angles for solution selection

        Returns:
            (success, joint_angles): bool and (4,) array or zeros on failure
        """
        px, py, pz = ee_frame[0, 3], ee_frame[1, 3], ee_frame[2, 3]
        R = ee_frame[:3, :3]

        L1 = self.A_2   # 700 mm — upper arm
        L2 = self.A_3   # 500 mm — forearm
        D1 = self.D_1   # 500 mm — base height

        # ── Step 1: theta_1 ────────────────────────────────────────────────────
        # R[0,2] = -sin(t1),  R[1,2] = cos(t1)
        s1 = -R[0, 2]
        c1 =  R[1, 2]
        if abs(s1) < 1e-9 and abs(c1) < 1e-9:
            # Degenerate: use position fallback
            rho = math.sqrt(px**2 + py**2)
            theta1_base = prev_joint_angles[0] if rho < 1e-6 else math.atan2(py, px)
        else:
            theta1_base = math.atan2(s1, c1)

        # Two theta_1 candidates: direct and flipped (arm pointing backward)
        theta1_candidates = [theta1_base]
        alt = theta1_base + math.pi
        if alt > math.pi:
            alt -= 2.0 * math.pi
        theta1_candidates.append(alt)

        # ── Steps 2–4: planar IK then wrist angle ──────────────────────────────
        solutions = []

        for t1 in theta1_candidates:
            if not self.joints[0].is_inside_joint_limit(t1):
                continue

            # Signed horizontal reach projected onto the arm's plane
            r = px * math.cos(t1) + py * math.sin(t1)

            # Vertical distance (positive = arm bends upward to reach target)
            h = D1 - pz   # = L1*sin(t2) + L2*sin(t2+t3)

            # Law of cosines → theta_3
            cos_t3 = (r**2 + h**2 - L1**2 - L2**2) / (2.0 * L1 * L2)
            if abs(cos_t3) > 1.0 + 1e-9:
                continue  # target out of reach
            cos_t3 = float(np.clip(cos_t3, -1.0, 1.0))

            for sign in [1, -1]:  # elbow-up (+) and elbow-down (-)
                t3 = sign * math.acos(cos_t3)
                if not self.joints[2].is_inside_joint_limit(t3):
                    continue

                # Solve theta_2
                beta  = math.atan2(h, r)
                gamma = math.atan2(L2 * math.sin(t3), L1 + L2 * math.cos(t3))
                t2 = beta - gamma
                if not self.joints[1].is_inside_joint_limit(t2):
                    continue

                # Solve theta_4 from total pitch sum encoded in R[2,:]
                # R[2,0] = -sin(t2+t3+t4),  R[2,1] = -cos(t2+t3+t4)
                t234 = math.atan2(-R[2, 0], -R[2, 1])
                t4 = t234 - t2 - t3

                # Wrap theta_4 to [-pi, pi]
                t4 = (t4 + math.pi) % (2.0 * math.pi) - math.pi

                if not self.joints[3].is_inside_joint_limit(t4):
                    continue

                solutions.append(np.array([t1, t2, t3, t4]))

        if not solutions:
            return False, np.zeros(4)

        # Pick solution closest to previous joint angles
        best = min(solutions, key=lambda s: np.linalg.norm(s - prev_joint_angles))

        # Verify with FK (position tolerance 1 mm)
        try:
            T_check = self.calculate_fk(best)
        except ValueError:
            return False, np.zeros(4)

        if np.linalg.norm(ee_frame[:3, 3] - T_check[:3, 3]) > 1.0:
            return False, np.zeros(4)

        return True, best

    # ── Drawing ────────────────────────────────────────────────────────────────

    def draw_robot(self, joint_angles: np.ndarray) -> None:
        """Draw the robot in the given configuration.

        Args:
            joint_angles: (4,) array of joint angles in radians
        """
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
        self.fig = plt.figure(figsize=(9, 9), facecolor='w')
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim([-ws, ws])
        self.ax.set_ylim([-ws, ws])
        self.ax.set_zlim([0, ws])
        self.ax.set_xlabel('X (mm)', fontsize=13)
        self.ax.set_ylabel('Y (mm)', fontsize=13)
        self.ax.set_zlabel('Z (mm)', fontsize=13)
        self.ax.set_title('RoboRoll Coatings — Painting Robot', fontsize=15)
        plt.grid(True)
