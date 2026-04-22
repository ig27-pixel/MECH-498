"""Student implementation for Q3 (3-DOF robot simulation)."""

import numpy as np

from RobBase import Trajectory
from RobSimulation import RobSimulation


class RobStudent(RobSimulation):

  def __init__(self, drawing_enabled=True):
    super().__init__(drawing_enabled=drawing_enabled)
    self._ik_angles = None
    self._int_err = np.zeros(3)
    self._int_started = False
    self._last_m4 = float(self.m4)

  def create_rob_trajectory(self, waypoints: np.ndarray) -> Trajectory:
    """Build a 30-second joint-space trajectory from 4 Cartesian waypoints."""
    total_duration = 30.0

    # Give the loaded lift toward WP2 a bit more time than the original split.
    t_dwell0_end = 2.0
    t_arrive1 = 9.0
    t_dwell1_end = 12.0
    t_arrive2 = 21.0
    t_dwell2_end = 26.0
    t_arrive3 = 29.0

    prev = np.array([0.0, np.radians(-20.0), np.radians(20.0)])
    ik_angles = []
    for wp in waypoints:
      wp_arr = np.asarray(wp, dtype=float)
      theta1_direct = np.arctan2(wp_arr[1], wp_arr[0])

      r_h = np.sqrt(wp_arr[0]**2 + wp_arr[1]**2)
      z_rel = wp_arr[2] - self.l1
      d2 = r_h**2 + z_rel**2
      cos_q3 = np.clip((d2 - self.l2**2 - self.l3**2) /
                       (2.0 * self.l2 * self.l3), -1.0, 1.0)
      q3_a = np.arccos(cos_q3)
      q3_b = -q3_a

      def _q2(q3v):
        k1 = self.l2 + self.l3 * np.cos(q3v)
        k2 = self.l3 * np.sin(q3v)
        return np.arctan2(z_rel, r_h) - np.arctan2(k2, k1)

      seeds = [
          prev,
          np.array([theta1_direct, prev[1], prev[2]]),
          np.array([theta1_direct + np.pi, -prev[1], prev[2]]),
          np.array([theta1_direct, _q2(q3_a), q3_a]),
          np.array([theta1_direct, _q2(q3_b), q3_b]),
      ]

      best_angles = prev.copy()
      best_err = float("inf")
      best_jdist = float("inf")
      for seed in seeds:
        success, angles = self.calculate_ik(wp_arr, seed)
        if not success:
          continue
        self.calculate_fk(angles)
        err = np.linalg.norm(self.ee_pos - wp_arr)
        jdist = np.linalg.norm(angles - prev)
        if err < best_err - 1.0 or (err <= best_err + 1.0 and jdist < best_jdist):
          best_err = err
          best_jdist = jdist
          best_angles = angles.copy()
      ik_angles.append(best_angles)
      prev = best_angles.copy()

    q0, q1, q2, q3 = ik_angles
    self._ik_angles = (q0, q1, q2, q3)
    self._t_seg = (t_dwell0_end, t_arrive1, t_dwell1_end,
                   t_arrive2, t_dwell2_end, t_arrive3)
    self._int_err[:] = 0.0
    self._int_started = False

    dt = self._dt
    n = int(round(total_duration / dt)) + 1
    timestamps = np.linspace(0.0, total_duration, n)

    def smoothstep5(alpha):
      alpha = np.clip(alpha, 0.0, 1.0)
      return alpha**3 * (10.0 - 15.0 * alpha + 6.0 * alpha**2)

    joint_poses = np.zeros((3, n))
    for i, t in enumerate(timestamps):
      if t < t_dwell0_end:
        q = q0.copy()
      elif t < t_arrive1:
        alpha = (t - t_dwell0_end) / (t_arrive1 - t_dwell0_end)
        q = q0 + smoothstep5(alpha) * (q1 - q0)
      elif t < t_dwell1_end:
        q = q1.copy()
      elif t < t_arrive2:
        alpha = (t - t_dwell1_end) / (t_arrive2 - t_dwell1_end)
        q = q1 + smoothstep5(alpha) * (q2 - q1)
      elif t < t_dwell2_end:
        q = q2.copy()
      elif t < t_arrive3:
        alpha = (t - t_dwell2_end) / (t_arrive3 - t_dwell2_end)
        q = q2 + smoothstep5(alpha) * (q3 - q2)
      else:
        q = q3.copy()
      joint_poses[:, i] = q

    joint_vels = np.zeros_like(joint_poses)
    for i in range(1, n - 1):
      joint_vels[:, i] = (joint_poses[:, i + 1] - joint_poses[:, i - 1]) / (2.0 * dt)

    for i, t in enumerate(timestamps):
      in_dwell = (t < t_dwell0_end or
                  t_arrive1 <= t < t_dwell1_end or
                  t_arrive2 <= t < t_dwell2_end or
                  t >= t_arrive3)
      if in_dwell:
        joint_vels[:, i] = 0.0

    traj = Trajectory()
    traj.set_timestamps(timestamps)
    traj.set_joint_1_poses(joint_poses[0])
    traj.set_joint_2_poses(joint_poses[1])
    traj.set_joint_3_poses(joint_poses[2])
    traj.set_joint_1_vels(joint_vels[0])
    traj.set_joint_2_vels(joint_vels[1])
    traj.set_joint_3_vels(joint_vels[2])
    return traj

  def get_rob_torque(self, theta: np.ndarray, theta_dot: np.ndarray,
                     timestep: float) -> np.ndarray:
    """Trajectory tracking with payload-aware gravity compensation."""
    t = float(timestep)
    if float(self.m4) != self._last_m4:
      self.calculate_parameters()
      self._last_m4 = float(self.m4)

    theta_ref, theta_dot_ref = self._get_desired_state(t)

    if self._ik_angles is not None:
      _, _, t1e, t2a, t2e, _ = self._t_seg
      if t < t1e:
        kp = np.array([180.0, 420.0, 180.0])
        kd = np.array([60.0, 160.0, 70.0])
      elif t < t2a:
        kp = np.array([300.0, 900.0, 380.0])
        kd = np.array([120.0, 320.0, 150.0])
      elif t < t2e:
        kp = np.array([380.0, 1100.0, 480.0])
        kd = np.array([150.0, 420.0, 180.0])
      else:
        kp = np.array([220.0, 600.0, 260.0])
        kd = np.array([80.0, 220.0, 100.0])
    else:
      t2a = t2e = -1.0
      kp = np.array([180.0, 420.0, 180.0])
      kd = np.array([60.0, 160.0, 70.0])

    c2 = np.cos(theta[1])
    c23 = np.cos(theta[1] + theta[2])
    distal_mass = self.m3 + self.m4
    gravity = np.array([
        0.0,
        self.g * 1e-3 * (self.m2 * (self.l2 / 2.0) * c2 +
                         distal_mass * (self.l2 * c2 + self.lc3 * c23)),
        self.g * 1e-3 * distal_mass * self.lc3 * c23,
    ])

    tau = (kp * (theta_ref - theta) +
           kd * (theta_dot_ref - theta_dot) +
           gravity +
           self.b * theta_dot)

    if self._ik_angles is not None and t2a <= t <= t2e:
      if not self._int_started:
        self._int_err[:] = 0.0
        self._int_started = True
      self._int_err += (theta_ref - theta) * self._dt
      self._int_err = np.clip(self._int_err, -0.2, 0.2)
      tau += np.array([8.0, 70.0, 35.0]) * self._int_err
    elif self._int_started:
      self._int_err[:] = 0.0
      self._int_started = False

    self._last_tau = tau
    return tau
