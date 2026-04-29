"""RobSimulation.py — Simulation infrastructure for the 3-DOF robot arm.

All dynamics math lives here; subclasses only need to implement
  create_rob_trajectory()  and  get_rob_torque().

_calculate_rob_dynamics() is the single injection point for the future
C++ .so module — no other file needs to change when that swap happens.
"""

import math
import numpy as np

from RobBase import RobBase, Trajectory
from rob_data import MotionData, Data

DRAW_EVERY = 10   # replay every N timesteps in _draw_simulation


class RobSimulation(RobBase):
  """3-DOF robot arm: Lagrangian dynamics + trajectory simulation loop.

  Subclasses must override:
    create_rob_trajectory(waypoints) -> Trajectory
    get_rob_torque(theta, theta_dot, t) -> np.ndarray (3,)
  """

  def __init__(self, drawing_enabled=True):
    super().__init__(drawing_enabled=drawing_enabled)
    self._traj     = None
    self._data     = Data()
    self._dt       = 0.01        # integration timestep [s]
    self._last_tau = np.zeros(3) # most recent torque — used by _calculate_rob_dynamics

  # ── Subclass interface ───────────────────────────────────────────────────────

  def create_rob_trajectory(self, waypoints):
    """Generate a joint-space Trajectory through the given EE waypoints.

    Must be overridden in subclasses.

    Args:
        waypoints (np.ndarray): (4, 3) array of EE positions (mm).

    Returns:
        Trajectory: populated Trajectory object.
    """
    raise NotImplementedError("create_rob_trajectory must be overridden")

  # ── Dynamics helpers ─────────────────────────────────────────────────────────

  def _calculate_rob_dynamics(self, t, state):
    """Forward dynamics via the compiled C++ module.

    Args:
        t (float): current simulation time [s]
        state (np.ndarray): (6,) = [θ1, θ2, θ3, θ̇1, θ̇2, θ̇3]

    Returns:
        np.ndarray: (3,) joint accelerations [rad/s²]
    """
    import rob_dynamics
    return rob_dynamics.calculate(state, self._last_tau, self.m4)

  # ── Simulation loop ──────────────────────────────────────────────────────────

  def _get_desired_state(self, t):
    """Interpolate the desired joint state from the stored trajectory at time t.

    Args:
        t (float): simulation time [s]

    Returns:
        tuple: (theta_des, theta_dot_des) each (3,) arrays
    """
    traj_times = np.array(self._traj.timesteps)
    t_clamped  = np.clip(t, traj_times[0], traj_times[-1])

    theta_des = np.array([
        np.interp(t_clamped, traj_times, self._traj.joint_1_poses),
        np.interp(t_clamped, traj_times, self._traj.joint_2_poses),
        np.interp(t_clamped, traj_times, self._traj.joint_3_poses),
    ])
    theta_dot_des = np.array([
        np.interp(t_clamped, traj_times, self._traj.joint_1_vels),
        np.interp(t_clamped, traj_times, self._traj.joint_2_vels),
        np.interp(t_clamped, traj_times, self._traj.joint_3_vels),
    ])
    return theta_des, theta_dot_des

  def _draw_simulation(self, waypoints, pickup_step, drop_step):
    """Replay the recorded simulation data as a frame-by-frame animation.

    Called only when drawing_enabled=True.  Never touches the physics.

    Args:
        waypoints (np.ndarray): (4, 3) EE waypoints — used for payload position.
        pickup_step (int): step index when ball was picked up  (-1 if never).
        drop_step   (int): step index when ball was dropped    (-1 if never).
    """
    if self._robot_payload is not None:
      self._robot_payload.set_position(waypoints[1])
      self._robot_payload.enable()

    for step, motion in enumerate(self._data):
      if step % DRAW_EVERY != 0:
        continue

      theta  = motion.theta
      ee_pos = motion.ee_frame[:3, 3]

      self.draw_rob(theta)

      if self._robot_payload is not None:
        if pickup_step >= 0 and drop_step >= 0:
          if step < pickup_step:
            self._robot_payload.set_position(waypoints[1])
          elif step < drop_step:
            self._robot_payload.set_position(ee_pos)
          else:
            self._robot_payload.set_position(waypoints[2])
        elif pickup_step >= 0 and step >= pickup_step:
          self._robot_payload.set_position(ee_pos)
        else:
          self._robot_payload.set_position(waypoints[1])
        self._robot_payload.draw()


  def simulate_rob(self, waypoints):
    """Run the full forward-dynamics simulation via the C++ module.

    Builds a trajectory, delegates the Euler integration loop to
    rob_dynamics.simulate(), then replays the animation and plots if
    drawing is enabled.

    Args:
        waypoints (np.ndarray): (4, 3) array of EE waypoints [mm].
    """
    import rob_dynamics
    self._data     = Data()
    self._last_tau = np.zeros(3)
    self._traj     = self.create_rob_trajectory(waypoints)
    result         = rob_dynamics.simulate(self, self._traj,
                                           np.asarray(waypoints, dtype=float))
    for r in result["records"]:
      self._data.append(r)
    if self._drawing_enabled:
      self._draw_simulation(waypoints,
                            result["pickup_step"],
                            result["drop_step"])
      self._data.plot_all(waypoints=waypoints)