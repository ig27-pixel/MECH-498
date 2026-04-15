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
from scipy.interpolate import interp1d

from RobBase import Trajectory
from RobSimulation import RobSimulation


class RobStudent(RobSimulation):

  def __init__(self, drawing_enabled=True):
    super().__init__(drawing_enabled=drawing_enabled)

  # ── Trajectory generation ─────────────────────────────────────────────────

  def create_rob_trajectory(self, waypoints: np.ndarray) -> Trajectory:
    """Build a 30-second joint-space trajectory from 4 Cartesian waypoints.

    Timing plan (30 s total):
      [0, 2)   — dwell at home (start)
      [2, 9)   — move home → ball_start
      [9, 11)  — dwell at ball_start  (robot settles; ball picked up)
      [11, 18) — move ball_start → ball_end
      [18, 20) — dwell at ball_end    (robot settles; ball dropped)
      [20, 27) — move ball_end → home
      [27, 30] — dwell at home (end)
    """
    total_duration = 30.0  # DO NOT CHANGE

    # ── Segment boundary times ──────────────────────────────────────────────
    t_dwell0_end   =  2.0   # end of initial dwell
    t_arrive1      =  9.0   # arrive at ball_start
    t_dwell1_end   = 11.0   # end of dwell at ball_start
    t_arrive2      = 18.0   # arrive at ball_end
    t_dwell2_end   = 20.0   # end of dwell at ball_end
    t_arrive3      = 27.0   # arrive at home (end)
    # 27 → 30 : final dwell at home

    # ── IK for each waypoint ────────────────────────────────────────────────
    # Chain prev angles so IK always picks the nearest reachable solution.
    prev = np.array([0.0, np.radians(-20.0), np.radians(20.0)])
    ik_angles = []
    for wp in waypoints:
      success, angles = self.calculate_ik(wp, prev)
      if not success:
        angles = prev.copy()
      ik_angles.append(angles)
      prev = angles.copy()

    q0, q1, q2, q3 = ik_angles  # home, ball_start, ball_end, home(end)

    # ── Build timestamp array ───────────────────────────────────────────────
    dt = self._dt                        # 0.01 s
    n  = int(round(total_duration / dt)) + 1
    timestamps = np.linspace(0.0, total_duration, n)

    # ── Interpolate joint angles ────────────────────────────────────────────
    joint_poses = np.zeros((3, n))
    for i, t in enumerate(timestamps):
      if t < t_dwell0_end:
        alpha = 0.0                          # dwell at home
        q = q0.copy()
      elif t < t_arrive1:
        alpha = (t - t_dwell0_end) / (t_arrive1 - t_dwell0_end)
        alpha = _smooth(alpha)
        q = q0 + alpha * (q1 - q0)
      elif t < t_dwell1_end:
        q = q1.copy()                        # dwell at ball_start
      elif t < t_arrive2:
        alpha = (t - t_dwell1_end) / (t_arrive2 - t_dwell1_end)
        alpha = _smooth(alpha)
        q = q1 + alpha * (q2 - q1)
      elif t < t_dwell2_end:
        q = q2.copy()                        # dwell at ball_end
      elif t < t_arrive3:
        alpha = (t - t_dwell2_end) / (t_arrive3 - t_dwell2_end)
        alpha = _smooth(alpha)
        q = q2 + alpha * (q3 - q2)
      else:
        q = q3.copy()                        # final dwell at home

      joint_poses[:, i] = q

    # ── Numerical velocity (central difference) ─────────────────────────────
    joint_vels = np.zeros_like(joint_poses)
    for i in range(1, n - 1):
      joint_vels[:, i] = (joint_poses[:, i + 1] - joint_poses[:, i - 1]) / (2.0 * dt)
    # endpoints: keep zero

    # Zero velocity during all dwell windows (robot must stop to pick/drop)
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
    """PD controller with gravity feed-forward.

    tau = -Kp*(theta - theta_ref) - Kd*(theta_dot - theta_dot_ref) + G(theta)
    """
    # ── Interpolate desired state from trajectory ────────────────────────────
    theta_ref     = np.zeros(3)
    theta_dot_ref = np.zeros(3)
    for i in range(3):
      interp_pos = interp1d(self._traj.raw[0].T, self._traj.raw[i + 1].T,
                            fill_value="extrapolate")
      interp_vel = interp1d(self._traj.raw[0],   self._traj.raw[i + 4],
                            fill_value="extrapolate")
      theta_ref[i]     = interp_pos(timestep)
      theta_dot_ref[i] = interp_vel(timestep)

    # ── Gravity compensation ─────────────────────────────────────────────────
    # Potential energy (with l in mm → convert to m):
    #   z_c2 = l1 + (l2/2)*sin(θ2)
    #   z_c3 = l1 + l2*sin(θ2) + lc3*cos(θ2+θ3)   [joint-3 offset = π/2]
    #
    # G[0] = ∂PE/∂θ1 = 0  (joint 1 rotates about vertical axis)
    # G[1] = g*1e-3 * [m2*(l2/2)*cos(θ2) + m3*(l2*cos(θ2) - lc3*sin(θ2+θ3))]
    # G[2] = g*1e-3 * m3 * (-lc3*sin(θ2+θ3))
    t2  = theta[1]
    t3  = theta[2]
    c2  = np.cos(t2)
    c23 = np.cos(t2 + t3)

    # FK: z_ee = l1 + l2*sin(θ2) + l3*sin(θ2+θ3)  (Modified DH convention)
    # → ∂PE/∂θ2 uses l2*cos(θ2)+lc3*cos(θ2+θ3), ∂PE/∂θ3 uses lc3*cos(θ2+θ3)
    G = np.array([
        0.0,
        self.g * 1e-3 * (self.m2 * (self.l2 / 2.0) * c2
                         + self.m3 * (self.l2 * c2 + self.lc3 * c23)),
        self.g * 1e-3 * self.m3 * self.lc3 * c23,
    ])

    # ── PD gains ─────────────────────────────────────────────────────────────
    # High gains ensure convergence within the 2-second dwell windows.
    Kp = np.array([2000.0, 12000.0, 6000.0])
    Kd = np.array([700.0,  2000.0,  1200.0])

    # ── Torque ───────────────────────────────────────────────────────────────
    pos_err = theta     - theta_ref
    vel_err = theta_dot - theta_dot_ref

    tau = -Kp * pos_err - Kd * vel_err + G
    return tau


# ── Helper ─────────────────────────────────────────────────────────────────────

def _smooth(alpha: float) -> float:
  """Smooth-step function: maps [0,1] → [0,1] with zero first derivative at ends."""
  alpha = np.clip(alpha, 0.0, 1.0)
  return alpha * alpha * (3.0 - 2.0 * alpha)
