"""RobStudent.py — Student implementation for Q3 (3-DOF robot simulation).

Inherits from RobSimulation and requires two methods to be implemented:

  create_rob_trajectory(waypoints)
      Build a joint-space trajectory from 4 Cartesian waypoints.

  get_rob_torque(theta, theta_dot, timestep)
      Compute joint torques at each simulation step (PD + gravity).

The simulation loop (simulate_rob) is provided by RobSimulation and will
call both of these methods automatically — you only need to implement them.
"""

from copy import deepcopy

import numpy as np

from RobBase import Trajectory
from RobSimulation import RobSimulation


class RobStudent(RobSimulation):

  def __init__(self, drawing_enabled=True):
    super().__init__(drawing_enabled=drawing_enabled)
    self._ik_angles = None   # (q0, q1, q2, q3) stored for controller
    self._ctrl_step = 0      # internal step counter for time tracking

  # ── Trajectory generation ─────────────────────────────────────────────────

  def create_rob_trajectory(self, waypoints: np.ndarray) -> Trajectory:
    """Build a 30-second joint-space trajectory from 4 Cartesian waypoints."""
    total_duration = 30.0  # DO NOT CHANGE

    # ── Segment boundary times ──────────────────────────────────────────────
    t_dwell0_end   =  3.0   # end of initial dwell
    t_arrive1      = 10.0   # arrive at ball_start
    t_dwell1_end   = 13.0   # end of dwell at ball_start
    t_arrive2      = 20.0   # arrive at ball_end
    t_dwell2_end   = 23.0   # end of dwell at ball_end
    t_arrive3      = 28.0   # arrive at home (end)
    # 28 → 30 : final dwell at home

    # ── IK for each waypoint ────────────────────────────────────────────────
    prev = np.array([0.0, np.radians(-20.0), np.radians(20.0)])
    ik_angles = []
    for wp in waypoints:
      success, angles = self.calculate_ik(wp, prev)
      if not success:
        angles = prev.copy()
      ik_angles.append(angles)
      prev = angles.copy()

    q0, q1, q2, q3 = ik_angles

    # Store for use in get_rob_torque (bypasses self._traj access)
    self._ik_angles = (q0, q1, q2, q3)
    self._t_seg = (t_dwell0_end, t_arrive1, t_dwell1_end,
                   t_arrive2, t_dwell2_end, t_arrive3)
    self._ctrl_step = 0  # reset step counter for new simulation

    # ── Build timestamp array ───────────────────────────────────────────────
    dt = self._dt
    n  = int(round(total_duration / dt)) + 1
    timestamps = np.linspace(0.0, total_duration, n)

    # ── Interpolate joint angles ────────────────────────────────────────────
    joint_poses = np.zeros((3, n))
    for i, t in enumerate(timestamps):
      if t < t_dwell0_end:
        q = q0.copy()
      elif t < t_arrive1:
        alpha = (t - t_dwell0_end) / (t_arrive1 - t_dwell0_end)
        alpha = alpha * alpha * (3.0 - 2.0 * alpha)
        q = q0 + alpha * (q1 - q0)
      elif t < t_dwell1_end:
        q = q1.copy()
      elif t < t_arrive2:
        alpha = (t - t_dwell1_end) / (t_arrive2 - t_dwell1_end)
        alpha = alpha * alpha * (3.0 - 2.0 * alpha)
        q = q1 + alpha * (q2 - q1)
      elif t < t_dwell2_end:
        q = q2.copy()
      elif t < t_arrive3:
        alpha = (t - t_dwell2_end) / (t_arrive3 - t_dwell2_end)
        alpha = alpha * alpha * (3.0 - 2.0 * alpha)
        q = q2 + alpha * (q3 - q2)
      else:
        q = q3.copy()
      joint_poses[:, i] = q

    # ── Numerical velocity (central difference) ─────────────────────────────
    joint_vels = np.zeros_like(joint_poses)
    for i in range(1, n - 1):
      joint_vels[:, i] = (joint_poses[:, i + 1] - joint_poses[:, i - 1]) / (2.0 * dt)

    # Zero velocity during all dwell windows
    for i, t in enumerate(timestamps):
      in_dwell = (t < t_dwell0_end or
                  t_arrive1 <= t < t_dwell1_end or
                  t_arrive2 <= t < t_dwell2_end or
                  t >= t_arrive3)
      if in_dwell:
        joint_vels[:, i] = 0.0

    # ── Assemble Trajectory ─────────────────────────────────────────────────
    traj = Trajectory()
    traj.set_timestamps(timestamps)
    traj.set_joint_1_poses(joint_poses[0])
    traj.set_joint_2_poses(joint_poses[1])
    traj.set_joint_3_poses(joint_poses[2])
    traj.set_joint_1_vels(joint_vels[0])
    traj.set_joint_2_vels(joint_vels[1])
    traj.set_joint_3_vels(joint_vels[2])
    return traj

  # ── Control law ───────────────────────────────────────────────────────────

  def get_rob_torque(self, theta: np.ndarray, theta_dot: np.ndarray,
                     timestep: float) -> np.ndarray:
    """Gravity feed-forward compensation (C++ handles PD internally)."""

    t2  = theta[1]
    t3  = theta[2]
    c2  = np.cos(t2)
    c23 = np.cos(t2 + t3)

    G = np.array([
        0.0,
        self.g * 1e-3 * (self.m2 * (self.l2 / 2.0) * c2
                         + self.m3 * (self.l2 * c2 + self.lc3 * c23)),
        self.g * 1e-3 * self.m3 * self.lc3 * c23,
    ])

    self._last_tau = G
    return G
