#!/usr/bin/env python3
import numpy as np
import math
import time

from fanuc import Fanuc


if __name__ == "__main__":

  robot = Fanuc()
  robot.draw_fanuc(np.array([math.pi/3, 0, 0, 0, 0, 0]))
  time.sleep(1.0)

  robot.draw_fanuc(np.array([math.pi/3, 0, 0, 0, 0, 0]))
  time.sleep(1.0)

  robot.draw_fanuc(np.array([math.pi/3, 0, 0, 0, 0, 0]))
  time.sleep(1.0)

  robot.draw_fanuc(np.array([math.pi/3, -math.pi/4, 0, 0, 0, 0]))
  time.sleep(1.0)
  