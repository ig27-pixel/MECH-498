import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import Tuple, List
from copy import deepcopy

import general_utility as general
from robot_components import Brush, Link, Joint


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
    """Initialize the workspace 

    Args:
        x_min (float): min x of the worksapce in mm
        x_max (float): max x of the workspace in mm 
        y_min (float): min y of the workspace in mm
        y_max (float): max y of the workspace in mm
        z_min (float): min z of the workspace in mm
        z_max (float): max z of the workspace in mm
    """
    self.x_min = x_min
    self.x_max = x_max
    self.y_min = y_min
    self.y_max = y_max
    self.z_min = z_min
    self.z_max = z_max


class Fanuc(object):
  """Fanuc class to hold the information about the Fanuc Arm """
  @property
  def joints(self) -> List[Joint]:
    return [
        self._joint_1, self._joint_2, self._joint_3, self._joint_4,
        self._joint_5, self._joint_6
    ]

  @property
  def ee_frame(self) -> np.ndarray:
    """Returns the position of the end effector given the joint transforms 

    Note, this is an inefficient call as it recalculates each time. If you want to 
    be fancy and make your code run faster, you can re-write this. 

    Returns:
        np.ndarray: (4,4) of the final location of the end effector. 
    """
    return self._ee_frame

  def __init__(self, drawing_enabled: bool = True):
    """Initialize the class """

    self._joints = []
    self._drawn_once = False
    self._drawing_enabled = drawing_enabled
    self._ee_frame = np.eye(4)


    """--------TODO------------"""
    self.a_0 = 0
    self.a_1 = 300
    self.a_2 = 900
    self.a_3 = 180
    self.a_4 = 0
    self.a_5 = 0
    
    self.l_1_z = 0
    self.l_2_z = 0
    self.l_3_z = 0
    self.l_4_z = 1600
    self.l_5_z = 0
    self.l_6_z = 180

    """^^^^^^^^^TODO^^^^^^^^^^^"""
    """--------TODO------------""" 
    self.workspace = Workspace(-3238,3238,-3238,3238,-3238,3238)
    """^^^^^^^^^TODO^^^^^^^^^^^"""


    # Initialzie the colors for drawing links. Feel free to change if you'd like.
    self.colors = [
        [0, 0, 0],
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
        [0, 1, 1],
        [1, 1, 0],
        [1, 0, 1],
    ]

    ## Create the base figure to pass to the joints and links
    if self._drawing_enabled:
      self._create_plot()
    else:
      self.fig = None
      self.ax = None

    ## Create the brush
    self.brush = Brush(self.ax)

    ## Make the base frame
    base_frame = make_frame([0, 0, 0], [0, 0, self.l_1_z])  #self.l_1_z
    self._base_frame = Joint(self.ax)
    self._base_frame.set_final_transform(base_frame)
    self._base_frame.set_dh_transform(base_frame)

    self._zero_frame = Joint(self.ax)
    self._zero_frame.set_dh_transform(np.eye(4))
    self._zero_frame.set_final_transform(np.eye(4))

    # Initialize and create the joints
    # Note -- do not fill in joint information here. Do that in _setup_joints()
    self._joint_1: Joint = None
    self._joint_2: Joint = None
    self._joint_3: Joint = None
    self._joint_4: Joint = None
    self._joint_5: Joint = None
    self._joint_6: Joint = None

    self._setup_joints()

    self._links = [Link(self.ax, self.colors[index]) for index in range(6)]

  def _setup_joints(self):
    """Use this area to initialize the joints.
    I have provided the start of the first joint. You need to
    - fill in the limits for the first joint. 
    - fill in all of the information for the remaining joints. 

    This will also help you get comfortable with initializing classes. 
    """

    self._joint_1 = Joint(self.ax, self.colors[0])
    self._joint_1.set_joint_limits(math.radians(-150), math.radians(150))
    self._joint_1.set_dh_value_a(self.a_0)
    self._joint_1.set_dh_value_alpha(self.l_1_z)
    self._joint_1.set_dh_value_d(0)

    self._joint_2 = Joint(self.ax, self.colors[0])
    self._joint_2.set_joint_limits(math.radians(-80), math.radians(80))
    self._joint_2.set_dh_value_a(self.a_1)
    self._joint_2.set_dh_value_alpha(math.radians(-90))
    self._joint_2.set_dh_value_d(self.l_2_z)

    self._joint_3 = Joint(self.ax, self.colors[0])
    self._joint_3.set_joint_limits(math.radians(-80), math.radians(80))
    self._joint_3.set_dh_value_a(self.a_2)
    self._joint_3.set_dh_value_alpha(math.radians(0))
    self._joint_3.set_dh_value_d(self.l_3_z)

    self._joint_4 = Joint(self.ax, self.colors[0])
    self._joint_4.set_joint_limits(math.radians(-240), math.radians(240))
    self._joint_4.set_dh_value_a(self.a_3)
    self._joint_4.set_dh_value_alpha(math.radians(-90))
    self._joint_4.set_dh_value_d(self.l_4_z)

    self._joint_5 = Joint(self.ax, self.colors[0])
    self._joint_5.set_joint_limits(math.radians(-120), math.radians(120))
    self._joint_5.set_dh_value_a(self.a_4)
    self._joint_5.set_dh_value_alpha(math.radians(90))
    self._joint_5.set_dh_value_d(self.l_5_z)

    self._joint_6 = Joint(self.ax, self.colors[0])
    self._joint_6.set_joint_limits(math.radians(-450), math.radians(450))
    self._joint_6.set_dh_value_a(self.a_5)
    self._joint_6.set_dh_value_alpha(math.radians(-90))
    self._joint_6.set_dh_value_d(self.l_6_z)
    # Note -- math.radians(value) will convert the degree input "value" into radians
    ... # next do joints 2 thru 6

    if self._drawing_enabled:
      for joint in self.joints:
        joint.draw()

  def calculate_fk(self, joint_angles: np.ndarray):
    """Calculate the forward kinematics of the fanuc. 

    In this function, you need to load the joint DH parameters into each joint. 
    This will then allow you to call self.ee_frame to get the final output 
    frame of the robot, or do any math using joint_X.dh_transform. 

    You must also raise a ValueExcpetion if the joint angles are out of range.
    for example: 
      if value > desired: 
        raise ValueError("value is too high")

    Args:
        joint_angles (np.ndarray): (1,6) array of the joint angles 
    Raises:
        ValueError: if the joint angles are out of range
    """

    # Joint limits check
    for i, (joint, theta) in enumerate(zip(self.joints, joint_angles), start=1):
        if not joint.is_inside_joint_limit(float(theta)):
            raise ValueError(f"Joint {i} angle {theta} is out of range")
      
    # Load the joint angles into the joints
    self._joint_1.set_theta(joint_angles[0])
    self._joint_2.set_theta(joint_angles[1] - (np.pi/2))
    self._joint_3.set_theta(joint_angles[2])
    self._joint_4.set_theta(joint_angles[3])
    self._joint_5.set_theta(joint_angles[4])
    self._joint_6.set_theta(joint_angles[5])

    # Chain the transforms together to get the final end effector frame.
    T = self._base_frame.dh_transform
    for joint in self.joints:
      T = T @ joint.dh_transform

    self._ee_frame = T
    return self._ee_frame
    

  def calculate_ik(self, ee_frame: np.ndarray,
                    prev_joint_angles: np.ndarray) -> Tuple[bool, np.ndarray]:
    """calculate the inverse kinematics of the fanuc
    This is the hard part of the whole thing. If you get this working, it's all downhill.
    Be careful, the devil is in the details. Be sure to include all solution cases. 
    Be sure to not make large leaps in configuration. 
    Be sure to test your code well. The auto-grader will certainly test many configurations of 
    your robot to ensure it can match them. 

    Args:
        ee_frame (np.ndarray): The desired location of the end effector in space as a 4x4 frame
        prev_joint_angles (np.ndarray): The previous joint angles (hint minimize the difference)

    Returns:
        Tuple[bool, np.ndarray]: bool -- whether or not a solutio exists
                                 np.ndarray -- the 6x1 array of that solution if it exists 
    """
    # Check if the desired end effector frame is within the workspace
    if not (self.workspace.x_min <= ee_frame[0, 3] <= self.workspace.x_max and
            self.workspace.y_min <= ee_frame[1, 3] <= self.workspace.y_max and
            self.workspace.z_min <= ee_frame[2, 3] <= self.workspace.z_max):
      return False, []
    
    # Initialize q with the previous joint angles, ensuring they are within limits.
    q = prev_joint_angles.copy()
    for i, joint in enumerate(self.joints):
      q[i] = np.clip(q[i], joint.low_limit, joint.high_limit)

    # Iterative IK to match ee_frame.
    for _ in range(300):
      T = self.calculate_fk(q)

      # Calculate the error in position and orientation.
      pos_err = ee_frame[:3, 3] - T[:3, 3]
      R_err = ee_frame[:3, :3] @ T[:3, :3].T
      rot_err = 0.5 * np.array([
          R_err[2, 1] - R_err[1, 2],
          R_err[0, 2] - R_err[2, 0],
          R_err[1, 0] - R_err[0, 1],
      ])
      err = np.hstack((pos_err, rot_err))

      # If the error is small enough, we have found a solution.
      if np.linalg.norm(pos_err) < 1e-3 and np.linalg.norm(rot_err) < 1e-4:
        break

      # Compute the Jacobian numerically.
      J = np.zeros((6, 6))
      for i, joint in enumerate(self.joints):
        step = 1e-5
        if q[i] + step > joint.high_limit:
          step = -step if q[i] - step >= joint.low_limit else 0.0
        if step == 0.0:
          continue

        qd = q.copy()
        qd[i] += step
        Td = self.calculate_fk(qd)

        dpos = (Td[:3, 3] - T[:3, 3]) / step
        Rd = Td[:3, :3] @ T[:3, :3].T
        drot = 0.5 * np.array([
            Rd[2, 1] - Rd[1, 2],
            Rd[0, 2] - Rd[2, 0],
            Rd[1, 0] - Rd[0, 1],
        ]) / step
        J[:, i] = np.hstack((dpos, drot))

      dq = np.linalg.solve(J.T @ J + max(1e-4, 1e-6 * np.linalg.norm(err)) * np.eye(6), J.T @ err)
      q = q + dq

      # Ensure joint limits are respected after each update.
      for i, joint in enumerate(self.joints):
        q[i] = np.clip(q[i], joint.low_limit, joint.high_limit)

    # Final check to ensure the solution is valid.
    T = self.calculate_fk(q)
    pos_err = ee_frame[:3, 3] - T[:3, 3]
    R_err = ee_frame[:3, :3] @ T[:3, :3].T
    rot_err = 0.5 * np.array([
        R_err[2, 1] - R_err[1, 2],
        R_err[0, 2] - R_err[2, 0],
        R_err[1, 0] - R_err[0, 1],
    ])

    if np.linalg.norm(pos_err) > 2e-3 or np.linalg.norm(rot_err) > 2e-3:
      return False, []

    return True, q


  def _create_plot(self):
    """Initialize the plot to use throughout
        No change necessary, setup function provided"""
    self.fig = plt.figure(figsize=(8, 8), facecolor='w')
    self.ax = self.fig.add_subplot(111, projection='3d')
    plt.xlim([self.workspace.x_min, self.workspace.x_max])
    plt.ylim([self.workspace.y_min, self.workspace.y_max])
    self.ax.set_zlim([self.workspace.z_min, self.workspace.z_max])
    self.ax.set_xlabel('X (mm)', fontsize=16)
    self.ax.set_ylabel('Y (mm)', fontsize=16)
    self.ax.set_zlabel('Z (mm)', fontsize=16)
    # self.ax.view_init(elev=22.8, azim=147.3)
    plt.grid(True)

  def initialize_fanuc_drawing(self, joint_angles: np.ndarray) -> None:
    """Initialize the drawing to ensure it draws properly
    No change needed, used to setup the drawing capability

    Args:
        joint_angles (np.ndarray): 6x1 array of joint angles desired. 
    """
    self.calculate_fk(joint_angles)
    plt.show(block=False)
    plt.pause(0.5)

  def draw_fanuc(self, joint_angles: np.ndarray) -> None:
    """Draw the fanuc in the provided configuration. 
    Note, this addes the distance from frame 0 to the mounting frame. 
    No change needed, provided drawing capability 

    Args:
        joint_angles (np.ndarray): 6x1 array of the desired joint angles. 
    """
    general.check_proper_numpy_format(joint_angles, (6, ))

    self.calculate_fk(joint_angles)
    if not self._drawn_once:
      self.initialize_fanuc_drawing(joint_angles)
      self._drawn_once = True

    self._base_frame.draw()
    self._zero_frame.draw()

    current_transform = self._base_frame.dh_transform

    for index, joint in enumerate(self.joints):
      prev_transform = current_transform
      current_transform = current_transform @ joint.dh_transform
      joint.set_final_transform(current_transform)
      joint.draw()
      self._links[index].update_frames(prev_transform, current_transform)
      self._links[index].draw()

    # draw the brush at the end
    self.brush.update_tool_frame(current_transform)
    self.brush.show_enabled()
    self.brush.paint()

    plt.pause(0.0001)
