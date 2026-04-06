"""rob_data.py — MotionData and Data containers for the 3-DOF robot simulation."""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class MotionData(object):
  """Snapshot of robot state at one simulation timestep."""

  def __init__(self, t, theta, theta_dot, theta_ddot, tau,
               theta_des, theta_dot_des, ee_frame):
    self.t            = t             # float      — simulation time          [s]
    self.theta        = theta         # (3,) array — joint positions           [rad]
    self.theta_dot    = theta_dot     # (3,) array — joint velocities          [rad/s]
    self.theta_ddot   = theta_ddot    # (3,) array — joint accelerations       [rad/s²]
    self.tau          = tau           # (3,) array — applied joint torques     [N·m]
    self.theta_des    = theta_des     # (3,) array — desired joint positions   [rad]
    self.theta_dot_des = theta_dot_des # (3,) array — desired joint velocities [rad/s]
    self.ee_frame     = ee_frame      # (4,4) array — EE homogeneous transform

  @property
  def ee_pos(self):
    """End-effector position [mm]."""
    return self.ee_frame[:3, 3]

  @property
  def pos_error(self):
    """Joint position tracking error [rad]."""
    return self.theta - self.theta_des

  @property
  def vel_error(self):
    """Joint velocity tracking error [rad/s]."""
    return self.theta_dot - self.theta_dot_des


class Data(object):
  """Container for all per-timestep MotionData records.

  Provides list-style access plus convenience array accessors for plotting
  and autograder checks.
  """

  def __init__(self):
    self._history = []

  def append(self, motion_data):
    self._history.append(motion_data)

  def __len__(self):
    return len(self._history)

  def __getitem__(self, idx):
    return self._history[idx]

  @property
  def history(self):
    return self._history

  # ── Convenience array accessors ──────────────────────────────────────────

  @property
  def times(self):
    """All timestamps as a 1-D array [s]."""
    return np.array([d.t for d in self._history])

  @property
  def thetas(self):
    """All joint positions as an (N, 3) array [rad]."""
    return np.array([d.theta for d in self._history])

  @property
  def theta_dots(self):
    """All joint velocities as an (N, 3) array [rad/s]."""
    return np.array([d.theta_dot for d in self._history])

  @property
  def theta_ddots(self):
    """All joint accelerations as an (N, 3) array [rad/s²]."""
    return np.array([d.theta_ddot for d in self._history])

  @property
  def taus(self):
    """All joint torques as an (N, 3) array [N·m]."""
    return np.array([d.tau for d in self._history])

  @property
  def thetas_des(self):
    """All desired joint positions as an (N, 3) array [rad]."""
    return np.array([d.theta_des for d in self._history])

  @property
  def theta_dots_des(self):
    """All desired joint velocities as an (N, 3) array [rad/s]."""
    return np.array([d.theta_dot_des for d in self._history])

  @property
  def ee_positions(self):
    """All EE positions as an (N, 3) array [mm]."""
    return np.array([d.ee_pos for d in self._history])

  @property
  def ee_frames(self):
    """All EE frames as an (N, 4, 4) array."""
    return np.array([d.ee_frame for d in self._history])

  @property
  def pos_errors(self):
    """All joint position tracking errors as an (N, 3) array [rad]."""
    return np.array([d.pos_error for d in self._history])

  # ── Plotting ─────────────────────────────────────────────────────────────

  def plot_joint_positions(self, waypoints=None, label=""):
    """Plot actual vs desired joint positions for all three joints.

    Args:
        waypoints (np.ndarray, optional): (4, 3) EE waypoints to overlay
            on a secondary panel showing the trajectory timing context.
        label (str): Figure title prefix.
    """
    if not self._history:
      return
    times = self.times
    thetas = self.thetas
    thetas_des = self.thetas_des

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle((label + " — " if label else "") + "Joint Positions")
    joint_names = [r'$\theta_1$ (yaw)', r'$\theta_2$ (shoulder)', r'$\theta_3$ (elbow)']

    for i, (ax, name) in enumerate(zip(axes, joint_names)):
      ax.plot(times, np.degrees(thetas[:, i]), color='tab:blue', label='actual')
      ax.plot(times, np.degrees(thetas_des[:, i]), color='tab:orange',
              linestyle='--', label='desired')
      ax.set_ylabel(f'{name} (deg)')
      ax.legend(loc='upper right', fontsize=8)
      ax.grid(True)

    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()

  def plot_joint_velocities(self, label=""):
    """Plot actual vs desired joint velocities for all three joints.

    Args:
        label (str): Figure title prefix.
    """
    if not self._history:
      return
    times = self.times
    theta_dots = self.theta_dots
    theta_dots_des = self.theta_dots_des

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle((label + " — " if label else "") + "Joint Velocities")
    joint_names = [r'$\dot\theta_1$', r'$\dot\theta_2$', r'$\dot\theta_3$']

    for i, (ax, name) in enumerate(zip(axes, joint_names)):
      ax.plot(times, np.degrees(theta_dots[:, i]), color='tab:blue', label='actual')
      ax.plot(times, np.degrees(theta_dots_des[:, i]), color='tab:orange',
              linestyle='--', label='desired')
      ax.set_ylabel(f'{name} (deg/s)')
      ax.legend(loc='upper right', fontsize=8)
      ax.grid(True)

    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()

  def plot_torques(self, tau_max=None, label=""):
    """Plot applied joint torques for all three joints.

    Args:
        tau_max (float, optional): If provided, draw ±tau_max limit lines.
        label (str): Figure title prefix.
    """
    if not self._history:
      return
    times = self.times
    taus = self.taus

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle((label + " — " if label else "") + "Joint Torques")
    joint_names = [r'$\tau_1$', r'$\tau_2$', r'$\tau_3$']

    for i, (ax, name) in enumerate(zip(axes, joint_names)):
      ax.plot(times, taus[:, i], color='tab:green')
      if tau_max is not None:
        ax.axhline( tau_max, color='red', linestyle='--', linewidth=0.8,
                    label=f'±{tau_max} N·m')
        ax.axhline(-tau_max, color='red', linestyle='--', linewidth=0.8)
        ax.legend(loc='upper right', fontsize=8)
      ax.set_ylabel(f'{name} (N·m)')
      ax.grid(True)

    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()

  def plot_ee_3d(self, waypoints=None, label=""):
    """Plot the 3-D end-effector trajectory.

    Args:
        waypoints (np.ndarray, optional): (4, 3) EE waypoint positions to
            mark on the trajectory as large markers.
        label (str): Figure title prefix.
    """
    if not self._history:
      return
    ee = self.ee_positions

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(ee[:, 0], ee[:, 1], ee[:, 2],
            color='tab:blue', linewidth=1.2, label='EE path')
    ax.scatter(ee[0, 0],  ee[0, 1],  ee[0, 2],
               color='green', s=60, zorder=5, label='start')
    ax.scatter(ee[-1, 0], ee[-1, 1], ee[-1, 2],
               color='red', s=60, zorder=5, label='end')

    if waypoints is not None:
      ax.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2],
                 color='orange', marker='*', s=120, zorder=6, label='waypoints')

    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title((label + " — " if label else "") + "EE Trajectory (3D)")
    ax.legend()
    plt.tight_layout()

  def plot_ee_vs_time(self, waypoints=None, waypoint_times=None, label=""):
    """Plot EE X, Y, Z position components vs time.

    Args:
        waypoints (np.ndarray, optional): (4, 3) EE waypoint positions.
        waypoint_times (array-like, optional): times when waypoints are
            expected to be reached, drawn as vertical dashed lines.
        label (str): Figure title prefix.
    """
    if not self._history:
      return
    times = self.times
    ee = self.ee_positions

    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    fig.suptitle((label + " — " if label else "") + "End-Effector Position vs Time")
    labels_xyz = ['X (mm)', 'Y (mm)', 'Z (mm)']
    colors_xyz = ['tab:red', 'tab:green', 'tab:blue']

    for i, (ax, lbl, col) in enumerate(zip(axes, labels_xyz, colors_xyz)):
      ax.plot(times, ee[:, i], color=col)
      if waypoints is not None:
        ax.axhline(waypoints[0, i], color='gray', linestyle=':', linewidth=0.8)
        for wp in waypoints:
          ax.axhline(wp[i], color='orange', linestyle='--', linewidth=0.8,
                     alpha=0.6)
      if waypoint_times is not None:
        for wt in waypoint_times:
          ax.axvline(wt, color='purple', linestyle=':', linewidth=0.8)
      ax.set_ylabel(lbl)
      ax.grid(True)

    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()

  def plot_tracking_error(self, label=""):
    """Plot joint position tracking error (actual − desired) vs time.

    Args:
        label (str): Figure title prefix.
    """
    if not self._history:
      return
    times = self.times
    errors = self.pos_errors

    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    fig.suptitle((label + " — " if label else "") + "Position Tracking Error")
    joint_names = [r'$\theta_1$ error', r'$\theta_2$ error', r'$\theta_3$ error']

    for i, (ax, name) in enumerate(zip(axes, joint_names)):
      ax.plot(times, np.degrees(errors[:, i]), color='tab:red')
      ax.axhline(0, color='black', linewidth=0.8, linestyle='--')
      ax.set_ylabel(f'{name} (deg)')
      ax.grid(True)

    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()

  def plot_all(self, waypoints=None, tau_max=None, label=""):
    """Comprehensive 6-panel diagnostic figure.

    Panels:
      Row 1: Joint positions (actual vs desired) | Joint velocities (actual vs desired)
      Row 2: Joint torques (all 3)               | 3-D EE trajectory
      Row 3: EE X, Y, Z vs time                 | Joint position tracking errors

    Args:
        waypoints (np.ndarray, optional): (4, 3) EE waypoints to mark.
        tau_max (float, optional): Torque limit lines.
        label (str): Figure suptitle.
    """
    if not self._history:
      return

    times       = self.times
    thetas      = self.thetas
    thetas_des  = self.thetas_des
    theta_dots  = self.theta_dots
    taus        = self.taus
    ee          = self.ee_positions
    errors      = self.pos_errors

    fig = plt.figure(figsize=(16, 12))
    fig.suptitle(label or "Simulation Diagnostic", fontsize=13)

    joint_colors  = ['tab:blue', 'tab:orange', 'tab:green']
    joint_labels  = [r'$\theta_1$', r'$\theta_2$', r'$\theta_3$']

    # ── Row 1, col 1: Joint positions ──────────────────────────────────────
    ax1 = fig.add_subplot(3, 2, 1)
    for i, (col, lbl) in enumerate(zip(joint_colors, joint_labels)):
      ax1.plot(times, np.degrees(thetas[:, i]),     color=col, label=f'{lbl} act.')
      ax1.plot(times, np.degrees(thetas_des[:, i]), color=col, linestyle='--',
               alpha=0.6, label=f'{lbl} des.')
    ax1.set_ylabel('Angle (deg)')
    ax1.set_title('Joint Positions')
    ax1.legend(fontsize=7, ncol=2)
    ax1.grid(True)

    # ── Row 1, col 2: Joint velocities ─────────────────────────────────────
    ax2 = fig.add_subplot(3, 2, 2)
    for i, (col, lbl) in enumerate(zip(joint_colors, joint_labels)):
      ax2.plot(times, np.degrees(theta_dots[:, i]), color=col, label=lbl)
    ax2.set_ylabel('Velocity (deg/s)')
    ax2.set_title('Joint Velocities')
    ax2.legend(fontsize=8)
    ax2.grid(True)

    # ── Row 2, col 1: Joint torques ────────────────────────────────────────
    ax3 = fig.add_subplot(3, 2, 3)
    for i, (col, lbl) in enumerate(zip(joint_colors, joint_labels)):
      ax3.plot(times, taus[:, i], color=col, label=lbl)
    if tau_max is not None:
      ax3.axhline( tau_max, color='red', linestyle='--', linewidth=0.8,
                   label=f'±{tau_max} N·m')
      ax3.axhline(-tau_max, color='red', linestyle='--', linewidth=0.8)
    ax3.set_ylabel('Torque (N·m)')
    ax3.set_title('Joint Torques')
    ax3.legend(fontsize=8)
    ax3.grid(True)

    # ── Row 2, col 2: 3-D EE trajectory ────────────────────────────────────
    ax4 = fig.add_subplot(3, 2, 4, projection='3d')
    ax4.plot(ee[:, 0], ee[:, 1], ee[:, 2],
             color='tab:blue', linewidth=1.2)
    ax4.scatter(ee[0, 0],  ee[0, 1],  ee[0, 2],
                color='green', s=40, zorder=5, label='start')
    ax4.scatter(ee[-1, 0], ee[-1, 1], ee[-1, 2],
                color='red',   s=40, zorder=5, label='end')
    if waypoints is not None:
      ax4.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2],
                  color='orange', marker='*', s=80, zorder=6, label='waypoints')
    ax4.set_xlabel('X (mm)', fontsize=8)
    ax4.set_ylabel('Y (mm)', fontsize=8)
    ax4.set_zlabel('Z (mm)', fontsize=8)
    ax4.set_title('EE Trajectory (3D)')
    ax4.legend(fontsize=7)

    # ── Row 3, col 1: EE position vs time ─────────────────────────────────
    ax5 = fig.add_subplot(3, 2, 5)
    for i, (comp, col) in enumerate(zip(['X', 'Y', 'Z'],
                                         ['tab:red', 'tab:green', 'tab:blue'])):
      ax5.plot(times, ee[:, i], color=col, label=comp)
    if waypoints is not None:
      for wp in waypoints:
        ax5.axhline(wp[0], color='tab:red',   linestyle=':', linewidth=0.6, alpha=0.5)
        ax5.axhline(wp[1], color='tab:green', linestyle=':', linewidth=0.6, alpha=0.5)
        ax5.axhline(wp[2], color='tab:blue',  linestyle=':', linewidth=0.6, alpha=0.5)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Position (mm)')
    ax5.set_title('EE Position vs Time')
    ax5.legend(fontsize=8)
    ax5.grid(True)

    # ── Row 3, col 2: Tracking error ──────────────────────────────────────
    ax6 = fig.add_subplot(3, 2, 6)
    for i, (col, lbl) in enumerate(zip(joint_colors, joint_labels)):
      ax6.plot(times, np.degrees(errors[:, i]), color=col, label=lbl)
    ax6.axhline(0, color='black', linewidth=0.8, linestyle='--')
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Error (deg)')
    ax6.set_title('Position Tracking Error')
    ax6.legend(fontsize=8)
    ax6.grid(True)

    plt.tight_layout()
