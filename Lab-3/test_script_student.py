#!/usr/bin/env python3
import numpy as np
import math
import time

from picasso import Picasso


def test_draw_faunc():
  robot = Picasso(drawing_enabled=True)

  input_values = np.array(
      [-math.pi / 6, math.pi / 5, -math.pi / 4, 1.25, -1.56, 7.25])
  robot.calculate_fk(input_values)


  starting_values = np.array([0, 0, 0, 0, 0, 0])

  test_path = "PATH_TO_FILE/teatra.yaml"

  robot.draw_picasso_path(starting_values, test_path)


if __name__ == "__main__":
  # load_data()

  robot = Picasso(drawing_enabled=True)

  robot.brush.selection = 1
  robot.draw_picasso(np.array([math.pi/3, -math.pi/4, 0, 0, 0, 0]))
  time.sleep(0.5)

  robot.brush.selection = 2
  robot.draw_picasso(np.array([math.pi/3, math.pi/4, 0, 0, 0, 0]))

  time.sleep(0.5)

  robot.brush.selection = 3
  robot.draw_picasso(np.array([-math.pi/3, math.pi/4, 0, 0, 0, 0]))

  time.sleep(0.5)

  test_draw_faunc()
  