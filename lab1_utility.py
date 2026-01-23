#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import lab1 as student
import general_utility as general_util
import yaml
import PyKDL as kdl


def draw_screw(translation: float,
               rotation: float,
               ax: np.ndarray,
               color: np.ndarray = None):
  # Check color input argument
  if color is not None:
    if np.shape(color) != (1, 3):
      raise ValueError('Input argument "color" must be a 3 element row vector')
    for i in range(3):
      if color[0, i] < 0 or color[0, i] > 1:
        raise ValueError(
            'All 3 elements of input argument "color" cannot be outside range 0 to 1'
        )
  else:
    color = np.array([0, 0, 1])  # set default color to blue

  # Create the screw transformation matrix
  screw_matrix = student.screw_tf(translation, rotation, ax)

  # Plot the reference frame
  draw_frame(screw_matrix, color)

  plt.title('Plot a Screw Transform', fontsize=20)
  plt.gca().set_aspect('equal', adjustable='box')
  plt.show()


def draw_frame(transform: np.ndarray, ax_input=None, color: np.ndarray = None):
  """Draw an individual Frame. If ax_input is not specified, it will generate a new graph
  If specified it will add to your current graph

  Args:
      transform (np.ndarray): 4x4 numpy array for the transformation matrix
      ax_input (_type_, optional): The plot to add to if a new one is not desired. Defaults to None.
      color (np.ndarray, optional): the color as a 3x1 array if desired. Defaults to None.
  """

  if ax_input is None:
    workspace = [-50, 50, -50, 50, -50, 50]
    fig = plt.figure(figsize=(8, 8), facecolor='w')
    ax = fig.add_subplot(111, projection='3d')
    plt.xlim(workspace[0:2])
    plt.ylim(workspace[2:4])
    ax.set_zlim(workspace[4:6])
    ax.set_xlabel('X (mm)', fontsize=16)
    ax.set_ylabel('Y (mm)', fontsize=16)
    ax.set_zlabel('Z (mm)', fontsize=16)
    ax.view_init(elev=22.8, azim=147.3)
    plt.grid(True)
  else:
    ax = ax_input

  kdl_frame = general_util.np_frame_to_kdl(transform)

  x = kdl_frame.p.x()
  y = kdl_frame.p.y()
  z = kdl_frame.p.z()

  x_axis = kdl.Vector(1, 0, 0)
  y_axis = kdl.Vector(0, 1, 0)
  z_axis = kdl.Vector(0, 0, 1)

  x_vector = kdl_frame.M * x_axis
  y_vector = kdl_frame.M * y_axis
  z_vector = kdl_frame.M * z_axis

  ax.quiver(x,
            y,
            z,
            x_vector.x(),
            x_vector.y(),
            x_vector.z(),
            length=50.0,
            color='r' if color is None else color)
  ax.quiver(x,
            y,
            z,
            y_vector.x(),
            y_vector.y(),
            y_vector.z(),
            length=50.0,
            color='g' if color is None else color)
  ax.quiver(x,
            y,
            z,
            z_vector.x(),
            z_vector.y(),
            z_vector.z(),
            length=50.0,
            color='b' if color is None else color)

  if ax_input is None:
    plt.show()


def draw_line_frame_to_frame(ax,
                             frame_1: np.ndarray,
                             frame_2: np.ndarray,
                             color=None):
  """Draw a line between two frames from first to second

  Args:
      ax (_type_): The figure to draw the line on
      frame_1 (np.ndarray): The 4x4 numpy array of the transformation for frame 1
      frame_2 (np.ndarray): The 4x4 numpy array of the transformation for frame 2
      color (_type_, optional): The desired color as a 3x1 matrix if desired. Defaults to None.
  """

  if color is None:
    color = 'black'

  frame_1 = general_util.np_frame_to_kdl(frame_1)
  frame_2 = general_util.np_frame_to_kdl(frame_2)

  ax.plot([frame_1.p.x(), frame_2.p.x()],
          [frame_1.p.y(), frame_2.p.y()],
          [frame_1.p.z(), frame_2.p.z()],
          color=color)


def draw_phantom(actuator_angles: np.ndarray,
                 gimbal_angles: np.ndarray,
                 ax_input=None) -> None:
  """Draw the phantom robot

  Args:
      actuator_angles (np.ndarray): 3x1 np array of the actuator angles
      gimbal_angles (np.ndarray): 3x1 np array of the gimbal angles

  Returns:
      None: None
  """
  colors = [
      [0.8, 0, 0],  # link 1 and frame 1 color
      [0, 0.8, 0],  # link 2 and frame 2 color
      [0, 0, 0.8],  # link 3 and frame 3 color
      [0.5, 0, 0.5],  # end effector frame color
      [0, 0.6, 0.6]  # gimbal frame color
  ]
  # Plotting workspace
  workspace = [-200, 200, -200, 200, -100, 300]

  # Convert actuator angles to joint angles
  joint_angles = student.actuator_to_joint(actuator_angles)

  # Create structure of PHANToM forward kinematics transforms
  _, phantom_T = student.phantom_fk(joint_angles, gimbal_angles)

  if ax_input is None:
    fig = plt.figure(figsize=(8, 8), facecolor='w')
    ax = fig.add_subplot(111, projection='3d')
    plt.xlim(workspace[0:2])
    plt.ylim(workspace[2:4])
    ax.set_zlim(workspace[4:6])
    ax.set_xlabel('X (mm)', fontsize=16)
    ax.set_ylabel('Y (mm)', fontsize=16)
    ax.set_zlabel('Z (mm)', fontsize=16)
    ax.view_init(elev=22.8, azim=147.3)
    plt.grid(True)
  else:
    ax = ax_input

  origin = np.eye(4)

  draw_frame(origin, ax)

  current_transform = origin

  for index, frame in enumerate(phantom_T):
    prev_transform = current_transform
    current_transform = current_transform @ frame
    if index > 0:
      draw_line_frame_to_frame(ax, prev_transform, current_transform,
                               colors[index])
    draw_frame(current_transform, ax, colors[index])

  if ax_input is None:
    plt.show()


def load_path_file(path_file: str) -> np.ndarray:
  """Load the path file into a dictionary 6xn

  Args:
      path_file (str): full path to file

  Returns:
      np.ndarray: 6xn array of actuator angles
  """
  with open(path_file, 'r') as file:
    path_data = yaml.safe_load(file)

  path_array = []
  for index, _ in enumerate(path_data["j1"]):
    path_array.append([
        path_data["j1"][index], path_data["j2"][index], path_data["j3"][index],
        path_data["j4"][index], path_data["j5"][index], path_data["j6"][index]
    ])

  return np.array(path_array)


def move_phantom(path_file):
  # Load path data
  data = load_path_file(path_file)
  s = data.T  # Transpose to have time steps along columns

  workspace = [-200, 200, -200, 200, -100, 300]

  # Draw PHANToM initially in zero position
  actuator_angles = np.array([0, 0, 0])
  gimbal_angles = np.array([0, 0, 0])

  plt.ion()
  fig = plt.figure(figsize=(8, 8), facecolor='w')
  ax = fig.add_subplot(111, projection='3d')
  plt.xlim(workspace[0:2])
  plt.ylim(workspace[2:4])
  ax.set_zlim(workspace[4:6])
  ax.set_xlabel('X (mm)', fontsize=16)
  ax.set_ylabel('Y (mm)', fontsize=16)
  ax.set_zlabel('Z (mm)', fontsize=16)
  ax.view_init(elev=22.8, azim=147.3)
  plt.grid(True)
  handles = draw_phantom(actuator_angles, gimbal_angles, ax)
  plt.show()

  # Draw in 3D
  for t in range(s.shape[1]):
    ax.clear()
    plt.xlim(workspace[0:2])
    plt.ylim(workspace[2:4])
    ax.set_zlim(workspace[4:6])
    ax.set_xlabel('X (mm)', fontsize=16)
    ax.set_ylabel('Y (mm)', fontsize=16)
    ax.set_zlabel('Z (mm)', fontsize=16)
    ax.view_init(elev=22.8, azim=147.3)
    plt.grid(True)

    # Move robot
    actuator_angles = s[0:3, t]
    gimbal_angles = s[3:6, t]
    joint_angles = student.actuator_to_joint(actuator_angles)
    phantom_T_0_g, phantom_T = student.phantom_fk(joint_angles, gimbal_angles)

    # Plot a point at the wrist for each step
    plt.plot([phantom_T_0_g[0, 3]], [phantom_T_0_g[1, 3]],
             [phantom_T_0_g[2, 3]],
             '.b',
             markersize=10.0)

    draw_phantom(actuator_angles, gimbal_angles, ax)
    plt.draw()
    plt.pause(0.001)

  plt.show()
  plt.ioff()
