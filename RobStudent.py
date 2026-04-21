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
    self._ik_angles = None   # (q0, q1, q2, q3) stored for controller
    self._ctrl_step = 0      # internal step counter for time tracking
    self._int_err = np.zeros(3)   # integral error accumulator
    self._int_started = False     # flag to reset integral at WP2 arrival

  # ── Trajectory generation ─────────────────────────────────────────────────

  def create_rob_trajectory(self, waypoints: np.ndarray) -> Trajectory:
    """Build a 30-second joint-space trajectory from 4 Cartesian waypoints."""
    total_duration = 30.0  # DO NOT CHANGE

    # ── Segment boundary times ──────────────────────────────────────────────
    t_dwell0_end   =  2.0   # end of initial dwell
    t_arrive1      =  9.0   # arrive at ball_start
    t_dwell1_end   = 12.0   # end of dwell at ball_start
    t_arrive2      = 20.0   # arrive at ball_end
    t_dwell2_end   = 26.0   # end of dwell at ball_end
    t_arrive3      = 29.0   # arrive at home (end)

    # ── IK for each waypoint ────────────────────────────────────────────────
    prev = np.array([0.0, np.radians(-20.0), np.radians(20.0)])
    ik_angles = []
    for wp in waypoints:
      wp_arr = np.asarray(wp, dtype=float)
      # Try multiple seeds and keep the solution whose FK is closest to wp
      theta1_direct = np.arctan2(wp_arr[1], wp_arr[0])

      # Analytical 2R closed-form seeds (elbow-up and elbow-down)
      r_h   = np.sqrt(wp_arr[0]**2 + wp_arr[1]**2)
      z_rel = wp_arr[2] - self.l1
      d2    = r_h**2 + z_rel**2
      cos_q3 = np.clip((d2 - self.l2**2 - self.l3**2) /
                       (2.0 * self.l2 * self.l3), -1.0, 1.0)
      q3_a =  np.arccos(cos_q3)   # elbow-down (external angle)
      q3_b = -q3_a                 # elbow-up

      def _q2(q3v):
        k1 = self.l2 + self.l3 * np.cos(q3v)
        k2 = self.l3 * np.sin(q3v)
        return np.arctan2(z_rel, r_h) - np.arctan2(k2, k1)

      seeds = [
          prev,
          np.array([theta1_direct,        prev[1],   prev[2]]),
          np.array([theta1_direct + np.pi, -prev[1],  prev[2]]),
          np.array([theta1_direct,         _q2(q3_a), q3_a]),
          np.array([theta1_direct,         _q2(q3_b), q3_b]),
      ]
      best_angles, best_err = prev.copy(), float('inf')
      for seed in seeds:
        s, a = self.calculate_ik(wp_arr, seed)
        if not s:
          continue
        self.calculate_fk(a)
        err = np.linalg.norm(self.ee_pos - wp_arr)
        if err < best_err:
          best_err, best_angles = err, a.copy()
      ik_angles.append(best_angles)
      prev = best_angles.copy()

    q0, q1, q2, q3 = ik_angles

    # Store for use in get_rob_torque (bypasses self._traj access)
    self._ik_angles = (q0, q1, q2, q3)
    self._t_seg = (t_dwell0_end, t_arrive1, t_dwell1_end,
                   t_arrive2, t_dwell2_end, t_arrive3)
    self._ctrl_step = 0  # reset step counter for new simulation
    self._int_err = np.zeros(3)
    self._int_started = False

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
    """PD + gravity feed-forward torque."""
    t = float(timestep)

    if self._ik_angles is not None:
      q0, q1, q2, q3 = self._ik_angles
      t0e, t1, t1e, t2a, t2e, t3a = self._t_seg  # boundary times

      def _seg(qa, qb, t_start, t_end, t_cur):
        s = (t_cur - t_start) / (t_end - t_start)
        alpha = s * s * (3.0 - 2.0 * s)
        d_alpha = 6.0 * s * (1.0 - s) / (t_end - t_start)
        return qa + alpha * (qb - qa), (qb - qa) * d_alpha

      if t <= t0e:
        theta_ref, theta_dot_ref = q0.copy(), np.zeros(3)
      elif t <= t1:
        theta_ref, theta_dot_ref = _seg(q0, q1, t0e, t1, t)
      elif t <= t1e:
        theta_ref, theta_dot_ref = q1.copy(), np.zeros(3)
      elif t <= t2a:
        theta_ref, theta_dot_ref = _seg(q1, q2, t1e, t2a, t)
      elif t <= t2e:
        theta_ref, theta_dot_ref = q2.copy(), np.zeros(3)
      elif t <= t3a:
        theta_ref, theta_dot_ref = _seg(q2, q3, t2e, t3a, t)
      else:
        theta_ref, theta_dot_ref = q3.copy(), np.zeros(3)

      # Switch to higher gains after WP1 is confirmed (t1e=12s): robot is at
      # q1 with ~zero velocity, so the switch has no torque discontinuity.
      # Higher gains improve tracking during WP1→WP2 travel and WP2 dwell.
      if t >= t1e:
        Kp = np.array([300.0, 600.0, 300.0])
        Kd = np.array([100.0, 400.0, 200.0])
      else:
        Kp = np.array([150.0, 300.0, 150.0])
        Kd = np.array([ 40.0, 100.0,  50.0])
    else:
      theta_ref, theta_dot_ref = self._get_desired_state(t)
      Kp = np.array([150.0, 300.0, 150.0])
      Kd = np.array([ 40.0, 100.0,  50.0])

    c2  = np.cos(theta[1])
    c23 = np.cos(theta[1] + theta[2])
    G = np.array([
        0.0,
        self.g * 1e-3 * (self.m2 * (self.l2 / 2.0) * c2
                         + self.m3 * (self.l2 * c2 + self.lc3 * c23)),
        self.g * 1e-3 * self.m3 * self.lc3 * c23,
    ])

    tau = Kp * (theta_ref - theta) + Kd * (theta_dot_ref - theta_dot) + G

    # Integral action from WP2 onward: eliminates steady-state offset
    if self._ik_angles is not None and t >= t2a:
      if not self._int_started:
        self._int_err = np.zeros(3)
        self._int_started = True
      self._int_err += (theta_ref - theta) * self._dt
      self._int_err = np.clip(self._int_err, -1.0, 1.0)  # anti-windup
      Ki = np.array([20.0, 200.0, 100.0])
      tau = tau + Ki * self._int_err

    self._last_tau = tau
    return tau
