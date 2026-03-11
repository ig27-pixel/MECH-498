import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D

import general_utility as general
from drawing_helper import FrameDrawing, LinkDrawing

class Brush(object):
  """Class holding the data for a brush object"""
  @property
  def selection(self):
    return self._selection

  @selection.setter
  def selection(self, selection):
    """Property for setting the internal brush selection

    Args:
        selection (int): selection to set inside the class

    Raises:
        TypeError: An int must be provided
        ValueError: the value must be in the length of colors availables
    """
    if not isinstance(selection, int):
      raise TypeError("brush must be an integer")
    if selection > len(self._colors):
      raise ValueError(
          "Selected a brush color outside of the range [0, {}]".format(
              len(self._colors)))
    self._selection = selection

  @property
  def color(self):
    return self._colors[self._selection]

  @property
  def selected_brush_frame(self):
    """Return the end brush frame 

    Returns:
        np.ndarray: the final frame in space of the brush as a (4,4)
    """
    if self._selection == 0:
      return np.eye(4)
    else:
      return self._brush_frames[self._selection - 1]

  @property
  def selected_brush_frame_dh(self):
    """Provide the DH parameters for the selected brush

    Returns the previous brush frame if 0 is selected. 
    This is to allow smother transitions between brushes. If your unsure why 
    this is necessary, double check your code that you are putting the 
    BRUSH at the desired location from the file, not just the end effector tip. 
    (Eg, if your tetra isn't continuous, this might be your issue)

    Returns:
        np.ndarray: The DH parameters of the selected brush as a (4,4)
    """
    if self._selection == 0:
      return self._tool_relative_frames[self._previously_selected_brush_frame]
    else:
      self._previously_selected_brush_frame = self._selection
      return self._tool_relative_frames[self._selection - 1]

  @property
  def selected_color(self):
    """Return the selected brush color

    Returns:
        array: 1x3 array of the color 
    """
    if self._selection == 0:
      return [0, 0, 0]
    else:
      return self._colors[self._selection - 1]

  def __init__(self, ax: Axes3D, drawing_enabled: bool = True):
    """Initialize the brush class 

    Args:
        ax (Axes3D): Axes3D used to draw the brush elements
    """
    self._colors = [[0.4940, 0.1840, 0.5560], [0.9290, 0.6940, 0.1250],
                    [0.8500, 0.3250, 0.0980], [0, 0.4470, 0.7410]]
    self._drawn_once = False
    self._selection = 0
    self._tool_base_frame = np.eye(4)
    self._drawing_enabled = drawing_enabled

    ##  Tool dimensions in millimeters
    self.l_t_rad = 50  #[mm]
    self.l_t = 300  #[mm]
    self._previously_selected_brush_frame = 1

    self._ax = ax

    self._tool_relative_frames = []
    self._create_tool_relative_frames()
    self._brush_frames = [np.eye(4) for _ in range(4)]

    if self._drawing_enabled:
      self._tool_lines = [LinkDrawing(self._ax, color) for color in self._colors]
    else:
      self._tool_lines = [None for _ in self._colors]

  def _create_tool_relative_frames(self):
    """Create the relative tool frames"""
    self._tool_relative_frames.append(
        self._make_tool_frame([0, -math.pi / 4, 5 * math.pi / 4],
                              [-self.l_t_rad, 0, self.l_t]))
    self._tool_relative_frames.append(
        self._make_tool_frame([0, -math.pi / 4, 7 * math.pi / 4],
                              [-self.l_t_rad, 0, self.l_t]))
    self._tool_relative_frames.append(
        self._make_tool_frame([0, -math.pi / 4, math.pi / 4],
                              [-self.l_t_rad, 0, self.l_t]))
    self._tool_relative_frames.append(
        self._make_tool_frame([0, -math.pi / 4, 3 * math.pi / 4],
                              [-self.l_t_rad, 0, self.l_t]))

  def _make_tool_frame(self, rotation: np.ndarray,
                       translation: np.ndarray) -> np.ndarray:
    """Make a tool frame to show up how we want in space w.r.t the EE 

    Args:
        rotation (np.ndarray): Rotation of the tool as rotations around X, Y, Z 
        translation (np.ndarray): Translation of the tool in X, Y, Z

    Returns:
        np.ndarray: (4,4) array of the final transformation
    """
    tool_frame = np.eye(4) 
    trans_x = np.eye(4) 
    trans_x[0,3] = translation[0]
    trans_z = np.eye(4)
    trans_z[2,3] = translation[2]

    tool_frame = tool_frame @ general.yawT(rotation[2])
    tool_frame = tool_frame @ trans_x 
    tool_frame = tool_frame @ general.pitchT(rotation[1])
    tool_frame = tool_frame @ trans_z

    return tool_frame

  def update_tool_frame(self, frame: np.ndarray):
    """Update the base frame for the tool (the robot end effector frame)

    Args:
        frame (np.ndarray): (4,4) Transformation matrix to the robot end effector
    """
    general.check_proper_numpy_format(frame, (4, 4))

    self._tool_base_frame = frame
    self._update_tool_frames()

  def _update_tool_frames(self, enable_all: bool = False):
    """Internal update of all of the tool frames to draw the correct ones

    Args:
        enable_all (bool, optional): True if you want to draw all brushes at once. Defaults to False.
    """

    for index, frame in enumerate(self._tool_relative_frames):
      new_frame = self._tool_base_frame @ frame

      self._brush_frames[index] = new_frame

      if self._drawing_enabled and self._tool_lines[index] is not None:
        if enable_all:
          self._tool_lines[index].update_frames(self._tool_base_frame,
                                                self._brush_frames[index])
        elif not enable_all and np.allclose(self.selected_brush_frame,
                                            self._brush_frames[index]):
          self._tool_lines[index].update_frames(self._tool_base_frame,
                                                self._brush_frames[index])
        else:
          self._tool_lines[index].update_frames(self._tool_base_frame,
                                                self._tool_base_frame)

  def _draw(self, enable_all: bool = False):
    """Internal method to handle the different ways of drawing depending on enabled brushes

    Args:
        enable_all (bool, optional): True if you want to draw all brushes at once. Defaults to False.
    """
    if not self._drawing_enabled:
      return
    
    if not self._drawn_once:
      self._update_tool_frames(enable_all=True)
      self._drawn_once = True
      for tool_line in self._tool_lines:
        tool_line.draw()

    self._update_tool_frames(enable_all)
    for tool_line in self._tool_lines:
      tool_line.redraw()

  def show_all(self):
    """Show all of the brushes in space"""
    if not self._drawing_enabled:
      return
    self._draw(enable_all=True)

  def show_enabled(self):
    """Show only the enabled brush"""
    if not self._drawing_enabled:
      return
    self._draw(enable_all=False)

  def paint(self):
    """Draw the enabled brush and leave a 'paint spot' behind in space"""
    if not self._drawing_enabled:
      return 
    
    self._draw(enable_all=True)

    if self._selection != 0:
      self._ax.plot([self.selected_brush_frame[0, 3]],
                    [self.selected_brush_frame[1, 3]],
                    [self.selected_brush_frame[2, 3]],
                    ".b",
                    color=self.selected_color,
                    markersize=10)


class Link(object):
  """Class to hold information about and for a link"""
  def __init__(self, ax: Axes3D, color=None, drawing_enabled: bool = True):
    """Create a Link 
    Links contain LinkDrawings and act a pass through and information holders

    Args:
        ax (Axes3D): Axes3D to use to draw
        color (_type_, optional): Color to draw if rgb frame is not desired. Defaults to None.
    """
    self._frame_1 = np.eye(4)
    self._frame_2 = np.eye(4)
    self._drawing_enabled = drawing_enabled
    if self._drawing_enabled:
      self._link_drawings = LinkDrawing(ax, color)
    else:
      self._link_drawings = None
    self._drawn_once = False

  def update_frames(self, frame_1: np.ndarray, frame_2: np.ndarray):
    """Update the start and end frame of the link

    Args:
        frame_1 (np.ndarray): (4,4) Transformation matrix to the start point
        frame_2 (np.ndarray): (4,4) Transformation matrix to the end point. 
    """
    general.check_proper_numpy_format(frame_1, (4, 4))
    general.check_proper_numpy_format(frame_2, (4, 4))

    if self._drawing_enabled and self._link_drawings is not None:
      self._link_drawings.update_frames(frame_1, frame_2)

  def draw(self):
    """Draw the link"""
    if not self._drawing_enabled or self._link_drawings is None:
      return
    
    if not self._drawn_once:
      self._link_drawings.draw()
      self._drawn_once = True
    else:
      self._link_drawings.redraw()

  def redraw(self):
    """Redraw the link"""
    if not self._drawing_enabled or self._link_drawings is None:
      return
    self._link_drawings.redraw()


class Joint(object):
  """Hold the definition and information for each joint"""

  @staticmethod
  def dh_tf(alpha: float, a: float, d: float, theta: float) -> np.ndarray:
    """Create a DH transform using alpha, a, d, and theta

    Args:
        alpha (float): alpha angle in radians
        a (float): a in mm
        d (float): d in mm
        theta (float): theta in radians

    Returns:
        np.ndarray: (4,4) numpy array of the transformation matrix
    """
    c_theta = math.cos(theta)
    s_theta = math.sin(theta)
    c_alpha = math.cos(alpha)
    s_alpha = math.sin(alpha)

    T = np.array([[c_theta, -s_theta, 0, a],
                  [s_theta * c_alpha, c_theta * c_alpha, -s_alpha, -s_alpha * d],
                  [s_theta * s_alpha, c_theta * s_alpha, c_alpha, c_alpha * d],
                  [0, 0, 0, 1]])

    return T

  @property
  def low_limit(self):
    """Returns the joints low joint limit

    Returns:
        float: low joint limit
    """
    return self._joint_limit[0]

  @property
  def high_limit(self):
    """Return the joints high limit

    Returns:
        float: joints high limit
    """
    return self._joint_limit[1]

  @property
  def dh_transform(self):
    """Return the joints dh_transform

    Returns:
        np.ndarray: (4,4) array of the DH transform of the joint
    """
    return self._dh_transform

  @property
  def final_transform(self):
    """Returns the stored final transform of the joint

    Returns:
        np.ndarray: (4,4) final transform of the joint 
    """
    return self._final_transform

  def __init__(self, ax: Axes3D, color=None, drawing_enabled: bool = True):
    """Initialize the joint

    Args:
        ax (Axes3D): Axes3D used to draw the joint
        color (array, optional): Color if black isn't desired. Defaults to None.
    """
    self._joint_limit = [0, 0]
    self._drawn_once = False
    self._dh_transform = np.eye(4)
    self._final_transform = np.eye(4)
    self._color = color
    self._a = 0.0
    self._alpha = 0.0
    self._d = 0.0
    self._theta = 0.0
    self._drawing_enabled = drawing_enabled
    if self._drawing_enabled: 
      self._frame_drawing = FrameDrawing(ax, color)
    else:
      self._frame_drawing = None

  def set_joint_limits(self, low_limit: float, high_limit: float):
    """Set the low and high joint limits

    Args:
        low_limit (float): Low limit of the joint
        high_limit (float): High limit of the joint
    """
    self._joint_limit = [low_limit, high_limit]

  def set_dh_value_a(self, value: float) -> None:
    """Set the a value for the DH parameters of the joint

    Args:
        value (float): value of the a parameter for the dh table (in mm) 
    """
    self._a = value 

  def set_dh_value_alpha(self, value: float) -> None: 
    """Set the alpha value for the DH parameters of the joint 

    Args:
        value (float): value of alpha in radians
    """
    self._alpha = value 

  def set_dh_value_d(self, value: float) -> None: 
    """Set the d value for the DH parameters of the joint

    Args:
        value (float): Value to store for d (in mm) 
    """
    self._d = value 

  def set_theta(self, value: float) -> None: 
    """Set the current value of theta for the joint

    Args:
        value (float): Theta value in radians 
    """
    self._theta = value 
    self.update_dh_transform()

  def update_dh_transform(self) -> None:
    """Update the DH transform using the stored DH parameters"""
    self._dh_transform = Joint.dh_tf(self._alpha, self._a, self._d, self._theta)

    if self._drawing_enabled and self._frame_drawing is not None:
      self._update_drawing()

  def set_dh_transform(self, transform: np.ndarray):
    """Set the DH transform of the joint

    Args:
        transform (np.ndarray): (4,4) array of the complete transform
    """
    general.check_proper_numpy_format(transform, (4, 4))
    self._dh_transform = transform

    if self._drawing_enabled and self._frame_drawing is not None: 
      self._update_drawing()

  def set_final_transform(self, transform: np.ndarray):
    """Set the final transform in space (where it will get drawn)

    Args:
        transform (np.ndarray): (4,4) transform for location to draw in space
    """
    general.check_proper_numpy_format(transform, (4, 4))
    self._final_transform = transform

  def _update_drawing(self):
    """Update the artist with most recent data"""
    if not self._drawing_enabled:
      return
    
    self._frame_drawing.update_frame(self._final_transform)

  def draw(self):
    """Draw the frame"""
    if not self._drawing_enabled or self._frame_drawing is None:
      return  
    
    self._update_drawing()
    if not self._drawn_once:
      self._frame_drawing.draw()
      self._drawn_once = True
    else:
      self._frame_drawing.redraw()
  
  def redraw(self):
    """Redraw the frame"""
    if not self._drawing_enabled or self._frame_drawing is None:
      return
    self._update_drawing()
    self._frame_drawing.redraw()

  def is_inside_joint_limit(self, input_angle: float) -> bool:
    """Check if the angle is insdie the joint limits

    Args:
        input_angle (float): angle to check

    Returns:
        bool: True if inside the limits, false otherwise 
    """
    if not self.low_limit <= input_angle <= self.high_limit:
      return False
    else:
      return True
