import numpy as np
import time
from typing import List

from fanuc_provided import Fanuc
from robot_components import Brush
import general_utility as general

import matplotlib.pyplot as plt

class Picasso(Fanuc):
  def __init__(self, swap_sign: bool = False, drawing_enabled: bool = True):
    super().__init__(swap_sign=swap_sign, drawing_enabled=drawing_enabled)
    ## Fanuc brush selection
    self.brush = Brush(self.ax)
    

  def draw_picasso(self, joint_angles: np.ndarray) -> None:
    """Draw the picasso using the fanuc drawing method. 

    Args:
        joint_angles (np.ndarray): 6x1 array of the desired joint angles. 
    """
    if not self._drawing_enabled:
      return
    
    ee_frame = super().draw_fanuc(joint_angles)

    # draw the brush at the end
    self.brush.update_tool_frame(ee_frame)
    self.brush.show_enabled()
    self.brush.paint()
    plt.pause(0.00001)

  def get_ee_pose_from_brush(self, rotation: np.ndarray,
                             brush_pose: np.ndarray) -> np.ndarray:
    """Get the end effector pose from the desired pose of the brush. 

    Note: If you are confused why this blank function is here. Remember that the 
    test drawing files, and your drawing files, are the location of the Brush tip, NOT the 
    end effector frame. Therefore, you need to calculate that EE frame from the brush frame 
    to put into your IK solution. 
    (If your tetra or prism isn't continuous, it's because this isn't working right)

    Args:
        rotation (np.ndarray): (3,3) rotation matrix for the brush
        brush_pose (np.ndarray): 1x3 position in space of the brush (x,y,z)

    Returns:
        np.ndarray: (4,4) location of the end effector to get the brush at the desired location
    """
    general.check_proper_numpy_format(rotation, (3, 3))
    general.check_proper_numpy_format(brush_pose, (3, ))

    # Build the 4x4 homogeneous transform for the desired brush tip pose
    T_brush = np.eye(4)
    T_brush[:3, :3] = rotation      
    T_brush[:3, 3] = brush_pose     

    # The brush tip is offset from the end effector frame by a fixed transform (T_brush_dh) that depends on the selected brush.
    T_brush_dh = self.brush.selected_brush_frame_dh
    ee_pose = T_brush @ np.linalg.inv(T_brush_dh)
    return ee_pose
  
  def calculate_picasso_path(self, starting_angles: np.ndarray, path: str) -> List[np.ndarray]:
    """ Calculate the joint angles to draw a desired path without drawing it

        This is what the auto-grader will use to evaluate how your robot draws. You must take in the 
        inputs of starting angles and path and generate a 7x1 numpy array of joint angles and brunch color
        [x, y, z, roll, pitch, yaw, color] to draw the path properly. You must take into account the 
        changing brush colors, the offset of the brush pose (see get_ee_pose_from_brush) and the starting 
        angles to draw a smooth path for the robot. Large jumps will be penalized. 

    Args:
        starting_angles (np.ndarray): 1x6 starting angles of the robot 
        path (str): full length path from your home directory to the file 
                    eg "/home/lcfarrell/ME_498/lab2/test_paths/prism.yaml
    Returns:
        List[np.ndarray]: List of 7x1 numpy arrays of joint angles and brush color to draw the path
    """
    # Load waypoint data (x, y, z positions and color index per point) from the YAML file
    data = general.get_data_from_yaml(path)
    output_path = []

    x_vals = data['x']
    y_vals = data['y']
    z_vals = data['z']
    colors = data['color']

    # Keep the brush orientation fixed 
    rotation = np.eye(3)

    # Track the previous joint angles for IK seeding and smooth interpolation
    prev_angles = np.array(starting_angles, dtype=float)
    # Track the previous color to detect brush color changes 
    prev_color = 0

    for i in range(len(x_vals)):
      color = int(colors[i])
      brush_pose = np.array([x_vals[i], y_vals[i], z_vals[i]], dtype=float)

      if color != prev_color and prev_color != 0:
        # Select the new brush color and solve IK for the target pose
        self.brush.selection = color
        ee_pose = self.get_ee_pose_from_brush(rotation, brush_pose)
        success, q_new = self.calculate_ik(ee_pose, prev_angles)
        if not success:
          # If IK fails, stay at the current configuration rather than making a bad jump
          q_new = prev_angles.copy()

        # Compute how many interpolation steps are needed so no joint moves more than 4 deg per step
        max_diff_deg = np.degrees(np.max(np.abs(q_new - prev_angles)))
        n_steps = max(1, int(np.ceil(max_diff_deg / 4.0)))
        for step in range(1, n_steps + 1):
          alpha = step / n_steps
          q_interp = prev_angles + alpha * (q_new - prev_angles)
          output_path.append(np.array([*q_interp, 0]))
        prev_angles = q_new

      # Select the current brush color and compute the EE pose for this waypoint
      self.brush.selection = color
      ee_pose = self.get_ee_pose_from_brush(rotation, brush_pose)

      # Solve IK seeded from the previous joint angles to keep the motion smooth
      success, joint_angles = self.calculate_ik(ee_pose, prev_angles)
      if success:
        prev_angles = joint_angles
      # Append the [q1..q6, color] vector for this waypoint
      output_path.append(np.array([*prev_angles, color]))
      prev_color = color

    return output_path

  def draw_picasso_path(self, starting_angles: np.ndarray, path: str) -> None:
    """Draw the fanuc moving through a desired path

    Args:
        starting_angles (np.ndarray): 1x6 starting angles of the robot 
        path (str): full length path from your home directory to the file 
                    eg "/home/lcfarrell/ME_498/lab2/test_paths/prism.yaml"
    """

    path_to_draw = self.calculate_picasso_path(starting_angles, path)
    for angles in path_to_draw:
      self.brush.selection = int(angles[6])
      self.draw_picasso(np.array(angles[:6]))

if __name__ == "__main__":
  picasso = Picasso(swap_sign=False, drawing_enabled=True)
  picasso.draw_picasso_path(
      starting_angles=np.radians([0, -90, 90, 0, 0, 0]),
      path="azalea.yaml")

  # picasso.draw_picasso(
  #     joint_angles=np.radians([0, 0, 0, 0, 0, 0]))
  # time.sleep(2.0)
  # picasso.draw_picasso(
  #     joint_angles=np.radians([0, 0, 0, 0, 0, 0]))
  # time.sleep(2.0)