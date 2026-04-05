import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import Tuple, List

import general_utility as general
from robot_components import Link, Joint


def make_frame(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
  """Create a tool transform using rotation and translation via ZYX Euler Trans.

  Args:
      rotation (np.ndarray): numpy 3x1 array of rotations
      translate (np.ndarray): numpy 3x1 array of translations

  Returns:
      np.ndarray: 4x4 numpy transformation array
  """
  frame = np.eye(4)
  frame = frame @ general.yawT(rotation[2])
  frame = frame @ general.pitchT(rotation[1])
  frame = frame @ general.rollT(rotation[0])
  frame[:3, 3] = translation
  return frame


class Workspace(object):
  def __init__(self, x_min: float, x_max: float, y_min: float, y_max: float,
               z_min: float, z_max: float):
    self.x_min = x_min
    self.x_max = x_max
    self.y_min = y_min
    self.y_max = y_max
    self.z_min = z_min
    self.z_max = z_max


class Fanuc(object):
  """Fanuc class to hold the information about the Fanuc Arm"""

  @property
  def joints(self) -> List[Joint]:
    return [
        self._joint_1, self._joint_2, self._joint_3, self._joint_4,
        self._joint_5, self._joint_6
    ]

  @property
  def ee_frame(self) -> np.ndarray:
    """Returns the position of the end effector given the joint transforms.

    Returns:
        np.ndarray: (4,4) of the final location of the end effector.
    """
    return (self._joint_1.dh_transform @ self._joint_2.dh_transform
            @ self._joint_3.dh_transform @ self._joint_4.dh_transform
            @ self._joint_5.dh_transform @ self._joint_6.dh_transform)

  def __init__(self, drawing_enabled: bool = True, swap_sign: bool = False):
    """Initialize the class"""

    self._joints = []
    self._drawn_once = False
    self._swap_sign = swap_sign
    self._drawing_enabled = drawing_enabled

    ## Fanuc link lengths in millimeters
    self.l_1_z = 1000  #[mm]
    self.l_1_x = 300   #[mm]
    self.l_2_x = 900   #[mm]
    self.l_3_x = 180   #[mm]
    self.l_3_y = 1600  #[mm]
    self.l_6_z = 180   #[mm]

    self.l_1 = self.l_1_z
    self.a_1 = self.l_1_x
    self.a_2 = self.l_2_x
    self.a_3 = self.l_3_x
    self.d_4 = self.l_3_y
    self.d_6 = self.l_6_z

    ## Create the workspace
    radius_xy = self.a_1 + self.a_2 + self.d_4
    radius_z  = self.l_1_z + self.a_2 + self.a_3 + self.d_4
    self.workspace = Workspace(-radius_xy, radius_xy, -radius_xy, radius_xy,
                               0, radius_z)

    self.colors = [
        [0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1],
        [0, 1, 1], [1, 1, 0], [1, 0, 1],
    ]

    if drawing_enabled:
      self._create_plot()
    else:
      self.fig = None
      self.ax  = None

    ## Base frame
    base_frame = make_frame([0, 0, 0], [0, 0, self.l_1_z])
    self._base_frame = Joint(self.ax)
    self._base_frame.set_final_transform(base_frame)
    self._base_frame.set_dh_transform(base_frame)

    self._zero_frame = Joint(self.ax)
    self._zero_frame.set_dh_transform(np.eye(4))
    self._zero_frame.set_final_transform(np.eye(4))

    self._joint_1: Joint = None
    self._joint_2: Joint = None
    self._joint_3: Joint = None
    self._joint_4: Joint = None
    self._joint_5: Joint = None
    self._joint_6: Joint = None

    self._setup_joints(swap_sign=swap_sign)

    self._links = [Link(self.ax, self.colors[index]) for index in range(6)]

    self.initialize_fanuc_drawing()

  def _setup_joints(self, swap_sign: bool):
    self.l_1_z = 1000; self.l_1_x = 300; self.l_2_x = 900
    self.l_3_x = 180;  self.l_3_y = 1600; self.l_6_z = 180

    self.l_1 = self.l_1_z
    self.a_1 = self.l_1_x
    self.a_2 = self.l_2_x
    self.a_3 = self.l_3_x
    self.d_4 = self.l_3_y
    self.d_6 = self.l_6_z

    sign = -1 if swap_sign else 1

    self._joint_1 = Joint(self.ax, self.colors[0])
    self._joint_1.set_joint_limits(math.radians(-150), math.radians(150))
    self._joint_1.set_dh_value_alpha(0.0)
    self._joint_1.set_dh_value_a(0.0)
    self._joint_1.set_dh_value_d(0.0)

    self._joint_2 = Joint(self.ax, self.colors[1])
    self._joint_2.set_joint_limits(math.radians(-80), math.radians(80))
    self._joint_2.set_dh_value_alpha(-sign * math.pi / 2)
    self._joint_2.set_dh_value_a(self.a_1)
    self._joint_2.set_dh_value_d(0.0)

    self._joint_3 = Joint(self.ax, self.colors[2])
    self._joint_3.set_joint_limits(math.radians(-80), math.radians(80))
    self._joint_3.set_dh_value_alpha(0.0)
    self._joint_3.set_dh_value_a(self.a_2)
    self._joint_3.set_dh_value_d(0.0)

    self._joint_4 = Joint(self.ax, self.colors[3])
    self._joint_4.set_joint_limits(math.radians(-240), math.radians(240))
    self._joint_4.set_dh_value_alpha(-sign * math.pi / 2)
    self._joint_4.set_dh_value_a(self.a_3)
    self._joint_4.set_dh_value_d(self.d_4)

    self._joint_5 = Joint(self.ax, self.colors[4])
    self._joint_5.set_joint_limits(math.radians(-120), math.radians(120))
    self._joint_5.set_dh_value_alpha(sign * math.pi / 2)
    self._joint_5.set_dh_value_a(0.0)
    self._joint_5.set_dh_value_d(0.0)

    self._joint_6 = Joint(self.ax, self.colors[5])
    self._joint_6.set_joint_limits(math.radians(-450), math.radians(450))
    self._joint_6.set_dh_value_alpha(-sign * math.pi / 2)
    self._joint_6.set_dh_value_a(0.0)
    self._joint_6.set_dh_value_d(self.d_6)

    if self._drawing_enabled:
      for joint in self.joints:
        joint.draw()

  def calculate_fk(self, joint_angles: np.ndarray, swap_sign: bool = False):
    """Calculate the forward kinematics of the Fanuc.

    Args:
        joint_angles (np.ndarray): (1,6) array of the joint angles
    Raises:
        ValueError: if the joint angles are out of range
    """
    for index, angle in enumerate(joint_angles):
      if not self.joints[index].is_inside_joint_limit(angle):
        print(f"input angle outside of the limit for joint {index + 1}")
        raise ValueError("input angle outside of the limit")

    self._joint_1.set_theta(joint_angles[0])
    self._joint_2.set_theta(joint_angles[1] - math.pi / 2)
    self._joint_3.set_theta(joint_angles[2])
    self._joint_4.set_theta(joint_angles[3])
    self._joint_5.set_theta(joint_angles[4])
    self._joint_6.set_theta(joint_angles[5])

  def calculate_ik(self, ee_frame: np.ndarray,
                   prev_joint_angles: np.ndarray) -> Tuple[bool, np.ndarray]:
    """Calculate the inverse kinematics of the Fanuc.

    Delegates to the compiled C++ extension (fanuc_ik) for the algorithm.

    Args:
        ee_frame (np.ndarray): Desired 4x4 end-effector frame
        prev_joint_angles (np.ndarray): Previous joint angles (radians)

    Returns:
        Tuple[bool, np.ndarray]: (is_solution, 6-element joint angle array)
    """
    import fanuc_ik  # compiled C++ extension
    params = {
        'a1': self.a_1, 'a2': self.a_2, 'a3': self.a_3,
        'd4': self.d_4, 'd6': self.d_6,
        'limits': [(j.low_limit, j.high_limit) for j in self.joints],
    }
    return fanuc_ik.calculate_ik(
        np.asarray(ee_frame, dtype=float),
        np.asarray(prev_joint_angles, dtype=float),
        params)

  def _create_plot(self):
    if not self._drawing_enabled:
      return
    self.fig = plt.figure(figsize=(8, 8), facecolor='w')
    self.ax  = self.fig.add_subplot(111, projection='3d')
    plt.xlim([self.workspace.x_min, self.workspace.x_max])
    plt.ylim([self.workspace.y_min, self.workspace.y_max])
    self.ax.set_zlim([self.workspace.z_min, self.workspace.z_max])
    self.ax.set_xlabel('X (mm)', fontsize=16)
    self.ax.set_ylabel('Y (mm)', fontsize=16)
    self.ax.set_zlabel('Z (mm)', fontsize=16)
    plt.grid(True)

  def initialize_fanuc_drawing(self) -> None:
    if not self._drawing_enabled:
      return
    self.calculate_fk([0, 0, 0, 0, 0, 0])
    plt.show(block=False)
    self.fig.canvas.draw()
    self.draw_fanuc([0, 0, 0, 0, 0, 0])
    print("initialized fanuc drawing")
    self._drawn_once = True

  def draw_fanuc(self, joint_angles: np.ndarray) -> np.ndarray:
    """Draw the Fanuc in the provided configuration.

    Args:
        joint_angles (np.ndarray): 6x1 array of the desired joint angles.
    """
    if not self._drawing_enabled:
      return

    general.check_proper_numpy_format(joint_angles, (6,))

    self.calculate_fk(joint_angles)

    self._base_frame.draw()
    self._zero_frame.draw()

    current_transform = self._base_frame.dh_transform

    for index, joint in enumerate(self.joints):
      prev_transform    = current_transform
      current_transform = current_transform @ joint.dh_transform
      joint.set_final_transform(current_transform)
      self._links[index].update_frames(prev_transform, current_transform)
      if self._drawn_once:
        joint.redraw()
        self._links[index].redraw()
      else:
        joint.draw()
        self._links[index].draw()

    self.fig.canvas.draw()
    self.fig.canvas.flush_events()
    return current_transform
