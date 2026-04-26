"""RobStudent.py — Student implementation for Q3 (3-DOF robot simulation).

Inherits from RobSimulation and requires two methods to be implemented:

  create_rob_trajectory(waypoints)
      Build a joint-space trajectory from 4 Cartesian waypoints.

  get_rob_torque(theta, theta_dot, timestep)
      Compute joint torques at each simulation step (PD + gravity).

The simulation loop (simulate_rob) is provided by RobSimulation and will
call both of these methods automatically — you only need to implement them.
"""

import numpy as np

from RobBase import Trajectory
from RobSimulation import RobSimulation

class RobStudent(RobSimulation):

  def __init__(self, drawing_enabled=True):
    super().__init__(drawing_enabled=drawing_enabled)
    self._ik_angles = None
    self._home_waypoint = None
    self._int_err = np.zeros(3)
    self._int_started = False
    self._last_m4 = float(self.m4)

  def create_rob_trajectory(self, waypoints: np.ndarray) -> Trajectory:
    """Build a 30-second joint-space trajectory from 4 Cartesian waypoints.

    The simulation visits each waypoint in order: home → ball_start →
    ball_end → home.  Your trajectory must give the robot enough time to
    reach and settle at each waypoint before the 30-second budget expires.

    You must create an output array that spans 30 seconds and returns a Trajectory class
    The trajectory class can be found in RobBase.
    To create the trajectory class, you need to add timestamps, joint poses, and joint velocities.
    The simulation will access these values to simulate the dynamics that you must later control.

    Args:
        waypoints (np.ndarray): (4, 3) array of EE positions [mm].
            waypoints[0] — home (start)
            waypoints[1] — ball start  (robot picks up object here)
            waypoints[2] — ball end    (robot drops object here)
            waypoints[3] — home (end)

    Returns:
        Trajectory: A fully populated Trajectory object.

    Trajectory API (see RobBase.py):
        trajectory = Trajectory()
        trajectory.set_timestamps(timesteps)          # 1-D array of times [s]
        trajectory.set_joint_1_poses(q1_array)        # joint-1 angles [rad]
        trajectory.set_joint_2_poses(q2_array)        # joint-2 angles [rad]
        trajectory.set_joint_3_poses(q3_array)        # joint-3 angles [rad]
        trajectory.set_joint_1_vels(qd1_array)        # joint-1 velocities [rad/s]
        trajectory.set_joint_2_vels(qd2_array)        # joint-2 velocities [rad/s]
        trajectory.set_joint_3_vels(qd3_array)        # joint-3 velocities [rad/s]

    Tips:
           - Use np.linspace to interpolate between consecutive waypoints.
           - Add a dwell (repeated sampled) at the waypoints so the robot can settle

    Important:
        total_duration = 30.0  # seconds — DO NOT CHANGE
    """
    total_duration = 30.0

    # Reserve time windows for travel and waypoint dwells.
    t_dwell0_end = 1.5
    t_arrive1 = 5.5
    t_dwell1_end = 7.0
    t_arrive2 = 11.5
    t_dwell2_end = 13.0
    t_arrive3 = 25.0

    # Prefer the nominal home posture when multiple IK solutions exist.
    home_seed = np.array([0.0, np.radians(-20.0), np.radians(20.0)])
    home_waypoint = np.asarray(waypoints[0], dtype=float)
    p_x, p_y, p_z = home_waypoint
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

    # Collect valid home IK solutions and keep only unique ones.
    q0_candidates = []
    for idx in range(4):
      q = np.array([theta_1[idx], theta_2[idx], theta_3[idx]])
      if (self._joint_1.is_inside_joint_limit(q[0]) and
          self._joint_2.is_inside_joint_limit(q[1]) and
          self._joint_3.is_inside_joint_limit(q[2])):
        self.calculate_fk(q)
        if np.linalg.norm(self.ee_pos - home_waypoint) <= 1.0:
          if not any(np.allclose(q, old) for old in q0_candidates):
            q0_candidates.append(q)

    if q0_candidates:
      q0 = min(q0_candidates, key=lambda q: np.linalg.norm(q - home_seed))
    else:
      q0 = home_seed.copy()
    self._home_waypoint = home_waypoint.copy()

    # Solve the first task waypoint near the previous joint state.
    waypoint_1 = np.asarray(waypoints[1], dtype=float)
    p_x, p_y, p_z = waypoint_1
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
    q1_candidates = []
    for idx in range(4):
      q = np.array([theta_1[idx], theta_2[idx], theta_3[idx]])
      if (self._joint_1.is_inside_joint_limit(q[0]) and
          self._joint_2.is_inside_joint_limit(q[1]) and
          self._joint_3.is_inside_joint_limit(q[2])):
        self.calculate_fk(q)
        if np.linalg.norm(self.ee_pos - waypoint_1) <= 1.0:
          if not any(np.allclose(q, old) for old in q1_candidates):
            q1_candidates.append(q)
    if q1_candidates:
      q1 = min(q1_candidates, key=lambda q: np.linalg.norm(q - q0)).copy()
    else:
      solved, q1 = self.calculate_ik(waypoint_1, q0)
      if not solved:
        raise ValueError(f"No IK solution found for waypoint {waypoint_1}")
      q1 = q1.copy()

    # Solve the second task waypoint near the first waypoint solution.
    waypoint_2 = np.asarray(waypoints[2], dtype=float)
    p_x, p_y, p_z = waypoint_2
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
    q2_candidates = []
    for idx in range(4):
      q = np.array([theta_1[idx], theta_2[idx], theta_3[idx]])
      if (self._joint_1.is_inside_joint_limit(q[0]) and
          self._joint_2.is_inside_joint_limit(q[1]) and
          self._joint_3.is_inside_joint_limit(q[2])):
        self.calculate_fk(q)
        if np.linalg.norm(self.ee_pos - waypoint_2) <= 1.0:
          if not any(np.allclose(q, old) for old in q2_candidates):
            q2_candidates.append(q)
    if q2_candidates:
      q2 = min(q2_candidates, key=lambda q: np.linalg.norm(q - q1)).copy()
    else:
      solved, q2 = self.calculate_ik(waypoint_2, q1)
      if not solved:
        raise ValueError(f"No IK solution found for waypoint {waypoint_2}")
      q2 = q2.copy()

    # Reuse the original home solution when the last waypoint matches home.
    if np.linalg.norm(np.asarray(waypoints[3], dtype=float) - np.asarray(waypoints[0], dtype=float)) < 1e-6:
      q3 = q0.copy()
    else:
      waypoint_3 = np.asarray(waypoints[3], dtype=float)
      p_x, p_y, p_z = waypoint_3
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
      q3_candidates = []
      for idx in range(4):
        q = np.array([theta_1[idx], theta_2[idx], theta_3[idx]])
        if (self._joint_1.is_inside_joint_limit(q[0]) and
            self._joint_2.is_inside_joint_limit(q[1]) and
            self._joint_3.is_inside_joint_limit(q[2])):
          self.calculate_fk(q)
          if np.linalg.norm(self.ee_pos - waypoint_3) <= 1.0:
            if not any(np.allclose(q, old) for old in q3_candidates):
              q3_candidates.append(q)
      if q3_candidates:
        q3 = min(q3_candidates, key=lambda q: np.linalg.norm(q - q2)).copy()
      else:
        solved, q3 = self.calculate_ik(waypoint_3, q2)
        if not solved:
          raise ValueError(f"No IK solution found for waypoint {waypoint_3}")
        q3 = q3.copy()
    self._ik_angles = (q0, q1, q2, q3)
    self._t_seg = (t_dwell0_end, t_arrive1, t_dwell1_end,
                   t_arrive2, t_dwell2_end, t_arrive3)
    self._final_hold_started = False
    self._int_err[:] = 0.0
    self._int_started = False

    # Sample the full 30-second trajectory at the simulator timestep.
    dt = self._dt
    n = int(round(total_duration / dt)) + 1
    timestamps = np.linspace(0.0, total_duration, n)

    # Build piecewise-smooth joint references through the four waypoints.
    joint_poses = np.zeros((3, n))
    for i, t in enumerate(timestamps):
      if t < t_dwell0_end:
        q = q0.copy()
      elif t < t_arrive1:
        alpha = (t - t_dwell0_end) / (t_arrive1 - t_dwell0_end)
        alpha = np.clip(alpha, 0.0, 1.0)
        blend = alpha**3 * (10.0 - 15.0 * alpha + 6.0 * alpha**2)
        q = q0 + blend * (q1 - q0)
      elif t < t_dwell1_end:
        q = q1.copy()
      elif t < t_arrive2:
        alpha = (t - t_dwell1_end) / (t_arrive2 - t_dwell1_end)
        alpha = np.clip(alpha, 0.0, 1.0)
        blend = alpha**3 * (10.0 - 15.0 * alpha + 6.0 * alpha**2)
        q = q1 + blend * (q2 - q1)
      elif t < t_dwell2_end:
        q = q2.copy()
      elif t < t_arrive3:
        alpha = (t - t_dwell2_end) / (t_arrive3 - t_dwell2_end)
        alpha = np.clip(alpha, 0.0, 1.0)
        blend = alpha**3 * (10.0 - 15.0 * alpha + 6.0 * alpha**2)
        q = q2 + blend * (q3 - q2)
      else:
        q = q3.copy()
      joint_poses[:, i] = q

    # Estimate reference velocities from the joint-position samples.
    joint_vels = np.zeros_like(joint_poses)
    for i in range(1, n - 1):
      joint_vels[:, i] = (joint_poses[:, i + 1] - joint_poses[:, i - 1]) / (2.0 * dt)

    # Force zero desired velocity during each dwell segment.
    for i, t in enumerate(timestamps):
      in_dwell = (t < t_dwell0_end or
                  t_arrive1 <= t < t_dwell1_end or
                  t_arrive2 <= t < t_dwell2_end or
                  t >= t_arrive3)
      if in_dwell:
        joint_vels[:, i] = 0.0

    # Package the sampled references into the provided Trajectory class.
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
    """PD controller with gravity feed-forward.

    Called at every simulation timestep by simulate_rob.  Returns the
    joint torques to apply

    The function inputs are the current theta, theta_dot, and timestep.  You must combine
    this with where the robot SHOULD be, accessed via self._traj, calculate the current error in position
    and, using your gains, calculate the desired joint torques. 

    To find where you think the robot SHOULD be, I used this for loop to calculate an interperator, then 
    fed that interperter my current timestep 
    for i in range(0,3):
      inter_ref  = interp1d(self._traj.raw[0].T, self._traj.raw[i+1].T, fill_value="extrapolate")
      inter_ref_dot = interp1d(self._traj.raw[0], self._traj.raw[i+4], fill_value="extrapolate")
      theta_ref[i] = inter_ref(timestep)
      theta_dot_ref[i] = inter_ref_dot(timestep)

    using your reference values for theta and theta dot, you can calculate errors and create your controller. 
    
    Note: You may need to use different control values for P and D for different joints. 

    Args:
        theta (np.ndarray): (3,) current joint positions [rad].
        theta_dot (np.ndarray): (3,) current joint velocities [rad/s].
        timestep (float): current simulation time [s].

    Returns:
        np.ndarray: (3,) joint torques [N·m].

    Tip:
        - don't forget about gravity! 
    """
    # Refresh payload-dependent parameters if the carried mass changes.
    t = float(timestep)
    if float(self.m4) != self._last_m4:
      self.calculate_parameters()
      self._last_m4 = float(self.m4)

    # Read the desired joint state from the stored reference trajectory.
    theta_ref, theta_dot_ref = self._get_desired_state(t)

    pos_err = theta_ref - theta
    vel_err = theta_dot_ref - theta_dot

    # Use phase-dependent gains to balance travel speed and settling.
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
        kp = np.array([260.0, 720.0, 300.0])
        kd = np.array([110.0, 300.0, 130.0])
      else:
        kp = np.array([780.0, 1900.0, 880.0])
        kd = np.array([140.0, 380.0, 170.0])
    else:
      t2a = t2e = -1.0
      t3a = -1.0
      kp = np.array([160.0, 420.0, 170.0])
      kd = np.array([55.0, 150.0, 65.0])

    c2 = np.cos(theta[1])
    c23 = np.cos(theta[1] + theta[2])
    distal_mass = self.m3 + self.m4
    # Add gravity feed-forward so the PD terms do not fight static load alone.
    gravity = np.array([
        0.0,
        self.g * 1e-3 * (self.m2 * (self.l2 / 2.0) * c2 +
                         distal_mass * (self.l2 * c2 + self.lc3 * c23)),
        self.g * 1e-3 * distal_mass * self.lc3 * c23,
    ])

    # Start with joint-space PD plus gravity compensation.
    tau = (kp * pos_err +
           kd * vel_err +
           gravity)

    # Keep a small integral term only during the payload transfer segment.
    if self._ik_angles is not None and t2a <= t <= t2e:
      if not self._int_started:
        self._int_err[:] = 0.0
        self._int_started = True
      self._int_err += pos_err * self._dt
      self._int_err = np.clip(self._int_err, -0.05, 0.05)
      tau += np.array([2.0, 12.0, 5.0]) * self._int_err
    elif self._int_started:
      self._int_err[:] = 0.0
      self._int_started = False

    # During final home hold, use strong uncapped damping to ensure velocity settles.
    if self._ik_angles is not None and t >= t3a and self._home_waypoint is not None:
      q3 = self._ik_angles[3]
      self.calculate_fk(theta)
      ee_err_norm = np.linalg.norm(self._home_waypoint - self.ee_pos)
      if ee_err_norm < 50.0:
        kp_hold = np.array([300.0, 750.0, 350.0])
        kd_hold = np.array([500.0, 1300.0, 600.0])
        tau = gravity + kp_hold * (q3 - theta) - kd_hold * theta_dot

    self._last_tau = tau
    return tau
