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
    raise NotImplementedError("create_rob_trajectory must be implemented")

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
    raise NotImplementedError("get_rob_torque must be implemented")
