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
    # ── Interpolate desired state from trajectory ────────────────────────────
    theta_ref, theta_dot_ref = self._get_desired_state(timestep)

    # ── Gravity compensation ─────────────────────────────────────────────────
    # FK: z_ee = l1 + l2*sin(θ2) + l3*sin(θ2+θ3)
    # PE = g*1e-3 * (m2*z_c2 + m3*z_c3)  [1e-3 converts mm → m]
    # G[i] = ∂PE/∂θi
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

    # ── PD gains ─────────────────────────────────────────────────────────────
    # Gains are critically overdamped and numerically stable for dt=0.01 s.
    # Euler eigenvalue analysis (A=[[1,dt],[-Kp*dt/M, 1-Kd*dt/M]]) confirms
    # all |λ| < 1 for estimated M11≈12, M22≈13, M33≈1.63 kg·m².
    # Settling time (5 × slow time-constant Kd/Kp) < 2 s for all joints.
    Kp = np.array([2000.0, 3000.0, 1500.0])
    Kd = np.array([ 400.0,  700.0,  160.0])

    # ── Torque ───────────────────────────────────────────────────────────────
    pos_err = theta     - theta_ref
    vel_err = theta_dot - theta_dot_ref

    tau = -Kp * pos_err - Kd * vel_err + G
    return tau
