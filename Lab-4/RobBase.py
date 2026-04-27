import math
import numpy as np
from typing import Tuple, List
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from copy import deepcopy

from robot_components import Link, Joint, RobotPayload
import general_utility as general


class JointStates(object):
  """Holds joint state information for all three joints at one timestep."""

  def __init__(self, joint_1_pos: float, joint_2_pos: float,
               joint_3_pos: float, joint_1_vel: float, joint_2_vel: float,
               joint_3_vel: float):
    self.joint_1_pos = joint_1_pos
    self.joint_2_pos = joint_2_pos
    self.joint_3_pos = joint_3_pos
    self.joint_1_vel = joint_1_vel
    self.joint_2_vel = joint_2_vel
    self.joint_3_vel = joint_3_vel


class Trajectory(object):
  """Holds trajectory data to enable a well-defined simulation trajectory."""

  @property
  def timesteps(self):
    """Array of timestamps (s).

    Returns:
        np.ndarray: timestep array
    """
    return self._timesteps

  @property
  def joint_1_poses(self):
    """Array of joint 1 positions (rad).

    Returns:
        np.ndarray: joint 1 position array
    """
    return self._joint_1_poses

  @property
  def joint_2_poses(self):
    """Array of joint 2 positions (rad).

    Returns:
        np.ndarray: joint 2 position array
    """
    return self._joint_2_poses

  @property
  def joint_3_poses(self):
    """Array of joint 3 positions (rad).

    Returns:
        np.ndarray: joint 3 position array
    """
    return self._joint_3_poses

  @property
  def joint_1_vels(self):
    """Array of joint 1 velocities (rad/s).

    Returns:
        np.ndarray: joint 1 velocity array
    """
    return self._joint_1_vels

  @property
  def joint_2_vels(self):
    """Array of joint 2 velocities (rad/s).

    Returns:
        np.ndarray: joint 2 velocity array
    """
    return self._joint_2_vels

  @property
  def joint_3_vels(self):
    """Array of joint 3 velocities (rad/s).

    Returns:
        np.ndarray: joint 3 velocity array
    """
    return self._joint_3_vels

  @property
  def dt(self):
    """Time step between consecutive trajectory samples (s).

    Returns:
        float: timestep duration
    """
    return (self._timesteps[-1] - self._timesteps[0]) / len(self._timesteps)

  @property
  def raw(self):
    """All trajectory data as a 7×n array (t, q1, q2, q3, qd1, qd2, qd3).

    Returns:
        np.ndarray: 7×n array
    """
    if self._raw is None:
      self._raw = np.array([
          np.array(self._timesteps),
          np.array(self._joint_1_poses),
          np.array(self._joint_2_poses),
          np.array(self._joint_3_poses),
          np.array(self._joint_1_vels),
          np.array(self._joint_2_vels),
          np.array(self._joint_3_vels),
      ])
    return self._raw

  @property
  def joint_states_array(self):
    """List of JointStates objects, one per trajectory sample."""
    if len(self._joint_states_array) != len(self._joint_1_poses):
      for index in range(len(self._joint_1_poses)):
        self._joint_states_array.append(
            JointStates(
                self._joint_1_poses[index],
                self._joint_2_poses[index],
                self._joint_3_poses[index],
                self._joint_1_vels[index],
                self._joint_2_vels[index],
                self._joint_3_vels[index],
            ))
    return self._joint_states_array

  def __init__(self):
    self._timesteps = []
    self._joint_1_poses = []
    self._joint_2_poses = []
    self._joint_3_poses = []
    self._joint_1_vels = []
    self._joint_2_vels = []
    self._joint_3_vels = []
    self._joint_states_array = []
    self._raw = None

  def set_timestamps(self, timestamp_array: np.ndarray):
    """Set the timestep array.

    Args:
        timestamp_array (np.ndarray): array of timesteps (s)
    """
    if not isinstance(timestamp_array, (np.ndarray, list)):
      raise TypeError("Wrong Type")
    self._timesteps = timestamp_array

  def set_joint_1_poses(self, pose_array: np.ndarray):
    """Set the joint 1 position array.

    Args:
        pose_array (np.ndarray): array of joint 1 positions (rad)
    """
    if not isinstance(pose_array, (np.ndarray, list)):
      raise TypeError("Wrong Type")
    self._joint_1_poses = pose_array

  def set_joint_2_poses(self, pose_array: np.ndarray):
    """Set the joint 2 position array.

    Args:
        pose_array (np.ndarray): array of joint 2 positions (rad)
    """
    if not isinstance(pose_array, (np.ndarray, list)):
      raise TypeError("Wrong Type")
    self._joint_2_poses = pose_array

  def set_joint_3_poses(self, pose_array: np.ndarray):
    """Set the joint 3 position array.

    Args:
        pose_array (np.ndarray): array of joint 3 positions (rad)
    """
    if not isinstance(pose_array, (np.ndarray, list)):
      raise TypeError("Wrong Type")
    self._joint_3_poses = pose_array

  def set_joint_1_vels(self, vel_array: np.ndarray):
    """Set the joint 1 velocity array.

    Args:
        vel_array (np.ndarray): array of joint 1 velocities (rad/s)
    """
    if not isinstance(vel_array, (np.ndarray, list)):
      raise TypeError("Wrong Type")
    self._joint_1_vels = vel_array

  def set_joint_2_vels(self, vel_array: np.ndarray):
    """Set the joint 2 velocity array.

    Args:
        vel_array (np.ndarray): array of joint 2 velocities (rad/s)
    """
    if not isinstance(vel_array, (np.ndarray, list)):
      raise TypeError("Wrong Type")
    self._joint_2_vels = vel_array

  def set_joint_3_vels(self, vel_array: np.ndarray):
    """Set the joint 3 velocity array.

    Args:
        vel_array (np.ndarray): array of joint 3 velocities (rad/s)
    """
    if not isinstance(vel_array, (np.ndarray, list)):
      raise TypeError("Wrong Type")
    self._joint_3_vels = vel_array

  def __len__(self):
    """Return the number of trajectory samples."""
    return len(self._joint_1_poses)


class Workspace(object):
  """Axis-aligned bounding box describing the robot workspace (mm)."""

  def __init__(self, x_min: float, x_max: float, y_min: float, y_max: float,
               z_min: float, z_max: float):
    self.x_min = x_min
    self.x_max = x_max
    self.y_min = y_min
    self.y_max = y_max
    self.z_min = z_min
    self.z_max = z_max


class RobBase(object):
  """Base class for the 3-DOF robot arm."""

  @property
  def joints(self) -> List[Joint]:
    """All joints including the end-effector frame.

    Returns:
        List[Joint]: [joint_1, joint_2, joint_3, joint_ee]
    """
    return [self._joint_1, self._joint_2, self._joint_3, self._joint_ee]

  @property
  def ee_frame(self) -> np.ndarray:
    """End-effector pose as the product of all joint DH transforms.

    Returns:
        np.ndarray: 4×4 homogeneous transform
    """
    return (self._joint_1.dh_transform @ self._joint_2.dh_transform @
            self._joint_3.dh_transform @ self._joint_ee.dh_transform)

  @property
  def tool_tip(self) -> np.ndarray:
    """Alias for ee_frame.

    Returns:
        np.ndarray: 4×4 homogeneous transform
    """
    return self.ee_frame

  @property
  def ee_pos(self) -> np.ndarray:
    """End-effector position (mm).

    Returns:
        np.ndarray: (3,) position vector
    """
    return self.ee_frame[0:3, 3]

  @property
  def ee_rotation(self) -> np.ndarray:
    """End-effector rotation matrix.

    Returns:
        np.ndarray: 3×3 rotation matrix
    """
    return self.ee_frame[0:3, 0:3]

  def __init__(self, drawing_enabled: bool = True):
    """Initialize the robot.

    Args:
        drawing_enabled (bool): If True, create Matplotlib figure and draw
                                the robot during simulation. Defaults to True.
    """
    self._drawing_enabled = drawing_enabled
    self._drawn_once = False
    self._frame_size = 75.0
    self.colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [0.5, 0.5, 0]]

    # Physical parameters
    self.b = 10    # joint damping [N·m·s/rad]
    self.g = 9.81  # gravitational acceleration [m/s²]
    self.m1 = 20   # mass of link 1 [kg]
    self.m2 = 10   # mass of link 2 [kg]
    self.m3 = 10   # mass of link 3 [kg]
    self.m4 = 0    # payload mass at end-effector [kg]
    self.l1 = 1000 # length of link 1 [mm]
    self.l2 = 700  # length of link 2 [mm]
    self.l3 = 700  # length of link 3 [mm]
    self.calculate_parameters()

    self._dt = 0.01
    self._duration = 5.0

    r_xy = 2000
    r_z = 2500
    self.workspace = Workspace(-r_xy, r_xy, -r_xy, r_xy, -r_z / 4, r_z)

    self.fig = None
    self.ax = None
    self._robot_payload = None
    if self._drawing_enabled:
      self._create_plot()

    self._joint_1 = None
    self._joint_2 = None
    self._joint_3 = None
    self._joint_ee = None
    self._setup_joints()

    self._joint_1_des = 0.0
    self._joint_2_des = 0.0

    self._G = None  # gravity matrix — populated by subclass dynamics

    if self._drawing_enabled:
      self._links = [Link(self.ax, self.colors[i], drawing_enabled=True)
                     for i in range(4)]
      self._base_frame = Joint(self.ax, drawing_enabled=True)
      self._base_frame.set_final_transform(np.eye(4))
      self._base_frame.set_dh_transform()
    else:
      self._links = None
      self._base_frame = None

  def calculate_parameters(self):
    """Compute derived inertia and COM parameters from link masses/lengths."""
    # Distance from frame 3 origin to link 3 COM [mm]
    self.lc3 = self.l3 * (self.m3 / 2 + self.m4) / (self.m3 + self.m4)

    # Link rotational inertias about each joint axis [kg·m²]
    self.I1 = self.m1 * (self.l1 * 1e-3)**2 / 12
    self.I2 = self.m2 * (self.l2 * 1e-3)**2 / 12
    self.I3 = (self.m3 * (self.l3 * 1e-3)**2 / 12 +
               self.m3 * ((self.lc3 * 1e-3) - (self.l3 * 1e-3) / 2)**2 +
               self.m4 * ((self.l3 * 1e-3) - (self.lc3 * 1e-3))**2)

    # Actuator inertias [kg·m²]
    self.J1 = self.I1 / 2
    self.J2 = self.I2 / 4
    self.J3 = self.I3 / 4

  def _setup_joints(self) -> None:
    """Instantiate and configure all four joints with DH parameters."""
    ax = self.ax  # None when drawing is disabled

    self._joint_1 = Joint(ax, frame_size=self._frame_size,
                          drawing_enabled=self._drawing_enabled)
    self._joint_1.set_joint_limits(-math.pi, math.pi)
    self._joint_1.set_dh_parameters(0, 0, self.l1, 0)

    self._joint_2 = Joint(ax, frame_size=self._frame_size,
                          drawing_enabled=self._drawing_enabled)
    self._joint_2.set_joint_limits(-math.pi / 2, math.pi / 2)
    self._joint_2.set_dh_parameters(0, math.pi / 2, 0, 0)

    self._joint_3 = Joint(ax, frame_size=self._frame_size,
                          drawing_enabled=self._drawing_enabled)
    self._joint_3.set_joint_limits(-math.pi, math.pi)
    self._joint_3.set_dh_parameters(self.l2, 0, 0, 0)

    self._joint_ee = Joint(ax, frame_size=self._frame_size,
                           drawing_enabled=self._drawing_enabled)
    self._joint_ee.set_joint_limits(-math.pi, math.pi)
    self._joint_ee.set_dh_parameters(0, math.pi / 2, self.l3, 0)

    if self._drawing_enabled:
      for joint in self.joints:
        joint.draw()

  def _create_plot(self) -> None:
    """Initialize the Matplotlib 3-D figure."""
    self.fig = plt.figure(figsize=(8, 8), facecolor='w')
    self.ax = self.fig.add_subplot(111, projection='3d')
    plt.xlim([self.workspace.x_min, self.workspace.x_max])
    plt.ylim([self.workspace.y_min, self.workspace.y_max])
    self.ax.set_zlim([self.workspace.z_min, self.workspace.z_max])
    self.ax.set_xlabel('X (mm)', fontsize=16)
    self.ax.set_ylabel('Y (mm)', fontsize=16)
    self.ax.set_zlabel('Z (mm)', fontsize=16)
    plt.grid(True)
    self._robot_payload = RobotPayload(self.ax)

  def calculate_fk(self, joint_angles: np.ndarray) -> None:
    """Update joint DH transforms for the given joint angles.

    Args:
        joint_angles (np.ndarray): (3,) array [θ1, θ2, θ3] in radians.
    """
    general.check_proper_numpy_format(joint_angles, (3,))
    self._joint_1.pos = joint_angles[0]
    self._joint_2.pos = joint_angles[1]
    self._joint_3.pos = joint_angles[2] + math.pi / 2

  def draw_rob(self, joint_angles: np.ndarray) -> None:
    """Render the robot at the given joint angles.

    Args:
        joint_angles (np.ndarray): (3,) array [θ1, θ2, θ3] in radians.
    """
    if not self._drawing_enabled:
      return

    general.check_proper_numpy_format(joint_angles, (3,))
    self.calculate_fk(joint_angles)

    if not self._drawn_once:
      plt.show(block=False)
      plt.pause(0.5)
      self._drawn_once = True

    self._base_frame.draw()
    current_transform = self._base_frame.dh_transform

    for index, joint in enumerate(self.joints):
      prev_transform = current_transform
      current_transform = current_transform @ joint.dh_transform
      joint.set_final_transform(current_transform)
      joint.draw()
      self._links[index].update_frames(prev_transform, current_transform)
      self._links[index].draw()

    plt.pause(0.001)

  def calculate_ik(self, desired_pose: np.ndarray,
                   prev_joint_angle: np.ndarray) -> Tuple[bool, np.ndarray]:
    """Compute inverse kinematics for the given end-effector position.

    Args:
        desired_pose (np.ndarray): (3,) desired EE position [x, y, z] in mm.
        prev_joint_angle (np.ndarray): (3,) previous joint angles (rad),
                                       used to select the nearest solution.

    Returns:
        Tuple[bool, np.ndarray]: (solution_found, joint_angles) where
            joint_angles is a (3,) array in radians, or None if no solution.
    """
    p_x = desired_pose[0]
    p_y = desired_pose[1]
    p_z = desired_pose[2]

    theta_1 = np.array([math.atan2(p_y, p_x), math.atan2(-p_y, -p_x)])
    p_x1 = p_x / np.cos(theta_1)
    p_z1 = p_z - self.l1

    c3 = (p_x1[0]**2 + p_z1**2 - self.l2**2 - self.l3**2) / (2 * self.l2 *
                                                                self.l3)
    s3 = np.array([math.sqrt(1 - c3**2), -math.sqrt(1 - c3**2)])
    theta_3 = np.array([math.atan2(s3[0], c3), math.atan2(s3[1], c3)])

    phi = np.array([math.atan2(p_z1, p_x1[0]), math.atan2(p_z1, p_x1[1])])
    beta = np.array([
        math.atan2(self.l3 * s3[0], self.l2 + self.l3 * c3),
        math.atan2(self.l3 * s3[1], self.l2 + self.l3 * c3),
    ])

    theta_1 = np.array([theta_1[0], theta_1[0], theta_1[1], theta_1[1]])
    theta_2 = np.array([
        phi[0] + beta[1], phi[0] + beta[0],
        phi[1] + beta[1], phi[1] + beta[0],
    ])
    theta_3 = np.array([theta_3[0], theta_3[1], theta_3[0], theta_3[1]])

    theta_1_good, theta_2_good, theta_3_good = [], [], []
    for idx in range(len(theta_1)):
      if (self._joint_1.is_inside_joint_limit(theta_1[idx]) and
          self._joint_2.is_inside_joint_limit(theta_2[idx]) and
          self._joint_3.is_inside_joint_limit(theta_3[idx])):
        theta_1_good.append(theta_1[idx])
        theta_2_good.append(theta_2[idx])
        theta_3_good.append(theta_3[idx])

    if not theta_1_good:
      print("Warning, no IK solution")
      return False, None

    min_diff = math.inf
    best_idx = 0
    for idx in range(len(theta_1_good)):
      diff = (abs(theta_1_good[idx] - prev_joint_angle[0]) +
              abs(theta_2_good[idx] - prev_joint_angle[1]) +
              abs(theta_3_good[idx] - prev_joint_angle[2]))
      if diff < min_diff:
        min_diff = diff
        best_idx = idx

    return True, np.array([
        theta_1_good[best_idx],
        theta_2_good[best_idx],
        theta_3_good[best_idx],
    ])

  def get_jacobian(self) -> np.ndarray:
    """Compute the 3×3 analytical Jacobian at the current joint configuration.

    Returns:
        np.ndarray: 3×3 Jacobian mapping joint velocities to EE linear velocity
    """
    s1 = math.sin(self._joint_1.pos)
    c1 = math.cos(self._joint_1.pos)
    s2 = math.sin(self._joint_2.pos)
    c2 = math.cos(self._joint_2.pos)
    s23 = math.sin(self._joint_2.pos + self._joint_3.pos)
    c23 = math.cos(self._joint_2.pos + self._joint_3.pos)
    l2 = self.l2 * 1e-3
    l3 = self.l3 * 1e-3

    return np.array([
        [-s1 * (c1 * l2 + c23 * l3), -c1 * (s2 * l2 + s23 * l3), -c1 * s23 * l3],
        [ c1 * (c1 * l2 + c23 * l3), -s1 * (s2 * l2 + s23 * l3), -s1 * s23 * l3],
        [0,                            c2 * l2 + c23 * l3,          c23 * l3],
    ])

  def create_waypoints(self, ball_start: np.ndarray,
                       ball_end: np.ndarray) -> np.ndarray:
    """Generate the four task waypoints for a ball-strike trajectory.

    The robot starts and ends at the home configuration (all joints = 0).

    Args:
        ball_start (np.ndarray): (3,) EE position at ball contact start (mm).
        ball_end   (np.ndarray): (3,) EE position at ball contact end (mm).

    Returns:
        np.ndarray: 4×3 array of waypoints [home, ball_start, ball_end, home]
    """
    self.calculate_fk(np.array([0, 0, 0]))
    home_pose = self.ee_pos.copy()
    return np.array([home_pose, ball_start, ball_end, home_pose])

  def create_rob_trajectory(self, waypoints: np.ndarray) -> Trajectory:
    """Generate a joint-space trajectory through the given waypoints.

    Must be overridden in subclasses.

    Args:
        waypoints (np.ndarray): 4×3 array of EE positions (mm).

    Returns:
        Trajectory: populated trajectory object.
    """
    raise NotImplementedError("create_rob_trajectory must be overridden")

  def get_rob_torque(self, theta: np.ndarray, theta_dot: np.ndarray,
                     timestep: float) -> np.ndarray:
    """Compute joint torques for the current state and time.

    Must be overridden in subclasses.

    Args:
        theta (np.ndarray): (3,) current joint positions (rad).
        theta_dot (np.ndarray): (3,) current joint velocities (rad/s).
        timestep (float): current simulation time (s).

    Returns:
        np.ndarray: (3,) joint torques (N·m).
    """
    raise NotImplementedError("get_rob_torque must be overridden")

  def simulate_rob(self, waypoints: np.ndarray):
    """Run the robot simulation.

    Implemented in RobSimulation; raises NotImplementedError here.

    Args:
        waypoints (np.ndarray): 4×3 array of EE waypoints (mm).
    """
    raise NotImplementedError("simulate_rob is implemented in RobSimulation")
