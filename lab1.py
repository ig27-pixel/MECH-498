#Isaiah Gonzalez lab1.py

import numpy as np
import PyKDL as kdl
import math
from typing import Tuple

def rotate(P_B: np.ndarray) -> np.ndarray:
  """Rotate a given vector by a set value

  Args:
      P_B (np.ndarray): Position Vector

  Returns:
      np.ndarray: rotated array 
  """  

  th_y = np.pi / 2
  th_z = np.pi / 4

  R_Y = np.array([[np.cos(th_y), 0, np.sin(th_y)],
                    [0, 1, 0],
                    [-np.sin(th_y), 0, np.cos(th_y)]
    ])
  
  R_Z = np.array([[np.cos(th_z), -np.sin(th_z), 0],
                    [np.sin(th_z), np.cos(th_z), 0],
                    [0, 0, 1]
    ])
  
  A_R_B = R_Y @ R_Z
  P_A = A_R_B @ P_B.T
  
  return P_A.T


def euler_to_ht(angles: np.ndarray, pos: np.ndarray) ->np.ndarray:
  """Create a transofrmation matrix and it's inverse using a ZYX Euler input

  Args:
      angles (np.ndarray): input angle vector
      pos (np.ndarray): input position vector

  Returns:
      np.ndarray: 4x4 transformation matrix created from position and angle vector
  """
  pass


def rollr(angle: float) -> np.ndarray:
  """Function that returns a 3x3 rotation matrix corresponding to roll

  Args:
      angle (float): angle to rotate by in radians
  Returns:
      np.ndarray: 3x3 rotation matrix 
  """
  pass


def pitchr(angle: float) -> np.ndarray:
  """Function that returns a 3x3 rotation matrix corresponding to pitch

  Args:
      angle (float): angle to rotate by in radians
  Returns:
      np.ndarray: 3x3 rotation matrix 
  """
  pass


def yawr(angle: float) -> np.ndarray:
  """Function that returns a 3x3 rotation matrix corresponding to yaw

  Args:
      angle (float): angle to rotate by in radians
  Returns:
      np.ndarray: 3x3 rotation matrix 
  """
  pass


def rpyr(angles: np.ndarray) -> np.ndarray:
  """Do roll pitch yaw for a 3 vector array of angles

  Args:
      angles (np.ndarray): (1,3) vector of angles

  Returns:
      np.ndarray: 3x3 rotaiton matrix
  """
  pass


def rpytf(values: np.ndarray) -> np.ndarray:
  """takes in a 1x6 vector of x,y,z,r,p,y to make a transformation matrix

  Args:
      values (np.ndarray): 1x6 array of x, y, z, roll, pitch, yaw

  Returns:
      np.ndarray: 4x4 Transformation Matrix 
  """
  pass


def screw_tf(translation: float, rotation: float, ax: np.ndarray) -> np.ndarray:
  """Create a Transformation Matrix using the angle-axis representation 

  Args:
      translation (float): translation along the axis
      rotation (float): rotation along the axis
      ax (np.ndarray): direction of the axis itself 

  Returns:
      np.ndarray: 4x4 Transformation matrix 
  """
  pass


def screw_dh(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
  """Create a frame using the DH Parameters 

  Args:
      a (float): distance from Zi to Zi+1 along the Xi
      alpha (float): angle from z to zi+1 about Xi
      d (float): diestance from Xi-1 to Xi along Zi
      theta (float): angle from Xi-1 to Xi measured about Zi

  Returns:
      np.ndarray: _description_
  """

  pass


def phantom_fk(joint_angles: np.ndarray,
               gimbal_angles: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
  """Create the FK for the phantom and return the full transform and all the homogenous matrices

  Args:
      joint_angles (np.ndarray): a 3x1 matrix of joint angles
      gimbal_angles (np.ndarray): a 3x1 matrix of gimbal angles for the end effector

  Returns: 
      Tuple[np.ndarray, np.ndarray]: 4x4 Full transformation matrix 
                                     All of the 4x4 homogeneous transformation matrices 
  """
  len_1 = 110.0 + 55
  len_2 = 205
  len_3 = 170.0

pass


def actuator_to_joint(actuator_angles: np.ndarray) -> np.ndarray:
  """Convert the actuator angles to joint angles

  Args:
      actuator_angles (np.ndarray): input actuator angles 3x1 np array

  Returns:
      np.ndarray: resulting joint angles as a 3x1 np array
  """

  ratio_1 = 13/175.0
  ratio_2 = 10.0/113.0
  ratio_3 = 10/113.0

  pass
