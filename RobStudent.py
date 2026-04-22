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

    t_dwell0_end = 1.5
    t_arrive1 = 5.5
    t_dwell1_end = 7.0
    t_arrive2 = 11.5
    t_dwell2_end = 13.0
    t_arrive3 = 20.0

    def all_ik_solutions(wp_arr: np.ndarray):
      p_x, p_y, p_z = wp_arr
      theta_1 = np.array([np.arctan2(p_y, p_x), np.arctan2(-p_y, -p_x)])
      p_x1 = p_x / np.cos(theta_1)
      p_z1 = p_z - self.l1

      c3 = (p_x1[0]**2 + p_z1**2 - self.l2**2 - self.l3**2) / (2.0 * self.l2 * self.l3)
      c3 = np.clip(c3, -1.0, 1.0)
      s3 = np.array([np.sqrt(max(0.0, 1.0 - c3**2)), -np.sqrt(max(0.0, 1.0 - c3**2))])
      theta_3 = np.array([np.arctan2(s3[0], c3), np.arctan2(s3[1], c3)])

      phi = np.array([np.arctan2(p_z1, p_x1[0]), np.arctan2(p_z1, p_x1[1])])
      beta = np.array([
          np.arctan2(self.l3 * s3[0], self.l2 + self.l3 * c3),
          np.arctan2(self.l3 * s3[1], self.l2 + self.l3 * c3),
      ])

      theta_1 = np.array([theta_1[0], theta_1[0], theta_1[1], theta_1[1]])
      theta_2 = np.array([
          phi[0] + beta[1], phi[0] + beta[0],
          phi[1] + beta[1], phi[1] + beta[0],
      ])
      theta_3 = np.array([theta_3[0], theta_3[1], theta_3[0], theta_3[1]])

      solutions = []
      for idx in range(4):
        q = np.array([theta_1[idx], theta_2[idx], theta_3[idx]])
        if (self._joint_1.is_inside_joint_limit(q[0]) and
            self._joint_2.is_inside_joint_limit(q[1]) and
            self._joint_3.is_inside_joint_limit(q[2])):
          self.calculate_fk(q)
          if np.linalg.norm(self.ee_pos - wp_arr) <= 1.0:
            if not any(np.allclose(q, old) for old in solutions):
              solutions.append(q)
      return solutions

    q0_candidates = all_ik_solutions(np.asarray(waypoints[0], dtype=float))
    if q0_candidates:
      home_seed = np.array([0.0, np.radians(-20.0), np.radians(20.0)])
      q0 = min(q0_candidates, key=lambda q: np.linalg.norm(q - home_seed))
    else:
      q0 = np.array([0.0, np.radians(-20.0), np.radians(20.0)])

    q3 = q0.copy() if np.linalg.norm(waypoints[3] - waypoints[0]) < 1e-6 else all_ik_solutions(
        np.asarray(waypoints[3], dtype=float))[0]

    q1_candidates = all_ik_solutions(np.asarray(waypoints[1], dtype=float))
    q2_candidates = all_ik_solutions(np.asarray(waypoints[2], dtype=float))

    best_cost = float("inf")
    q1 = q1_candidates[0]
    q2 = q2_candidates[0]
    for q1_cand in q1_candidates:
      for q2_cand in q2_candidates:
        d01 = np.linalg.norm(q1_cand - q0)
        d12 = np.linalg.norm(q2_cand - q1_cand)
        d23 = np.linalg.norm(q3 - q2_cand)
        cost = d01 + d12 + 4.0 * d23
        if cost < best_cost:
          best_cost = cost
          q1 = q1_cand.copy()
          q2 = q2_cand.copy()

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
    """PD trajectory tracking with gravity feed-forward."""
    t = float(timestep)
    if float(self.m4) != self._last_m4:
      self.calculate_parameters()
      self._last_m4 = float(self.m4)

    theta_ref, theta_dot_ref = self._get_desired_state(t)

    if self._ik_angles is not None:
      _, _, t1e, t2a, t2e, t3a = self._t_seg
      if t < t1e:
        kp = np.array([160.0, 420.0, 170.0])
        kd = np.array([55.0, 150.0, 65.0])
      elif t < t2a:
        kp = np.array([240.0, 650.0, 280.0])
        kd = np.array([80.0, 220.0, 95.0])
      elif t < t2e:
        kp = np.array([280.0, 760.0, 320.0])
        kd = np.array([95.0, 260.0, 110.0])
      elif t < t3a:
        kp = np.array([200.0, 560.0, 240.0])
        kd = np.array([90.0, 250.0, 110.0])
      else:
        kp = np.array([30.0, 85.0, 40.0])
        kd = np.array([320.0, 900.0, 380.0])
    else:
      t2a = t2e = -1.0
      t3a = -1.0
      kp = np.array([160.0, 420.0, 170.0])
      kd = np.array([55.0, 150.0, 65.0])

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
           gravity)

    if self._ik_angles is not None and t2a <= t <= t2e:
      if not self._int_started:
        self._int_err[:] = 0.0
        self._int_started = True
      self._int_err += (theta_ref - theta) * self._dt
      self._int_err = np.clip(self._int_err, -0.05, 0.05)
      tau += np.array([2.0, 12.0, 5.0]) * self._int_err
    elif self._int_started:
      self._int_err[:] = 0.0
      self._int_started = False

    self._last_tau = tau
    return tau
