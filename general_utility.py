import numpy as np
import math
import PyKDL as kdl
import yaml


def check_proper_numpy_format(value: np.ndarray, shape: tuple) -> bool:
  """Send in a numpy array to make sure it is the right shape 

  Args:
      value (np.ndarray): Input value to test
      shape (tuple): desired shape as a tuple ex (4,4)

  Returns:
      bool: If True, the array is well formed 
  """
  if not isinstance(value, np.ndarray):
    return False

  if value.shape != shape:
    return False

  return True


def np_frame_to_kdl(np_frame: np.ndarray) -> kdl.Frame:
  """Turn a numpy 4x4 array into a KDL frame 
  NOTE: It does NOT check that the 4x4 array is a proper transformation

  Args:
      np_frame (np.ndarray): Input numpy array of shape 4x4

  Returns:
      kdl.Frame: KDL version of that transformation
  """

  if not check_proper_numpy_format(np_frame, (4, 4)):
    raise TypeError("Incorret type or size to translate into a KDL Frame")

  kdl_frame = kdl.Frame()
  for i in range(3):
    for j in range(3):
      kdl_frame.M[i, j] = np_frame[i, j]
    kdl_frame.p[i] = np_frame[i, 3]

  return kdl_frame

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