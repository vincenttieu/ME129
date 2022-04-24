#!/usr/bin/env python3

from turtle import right
import pigpio
import sys
import time
import numpy as np
from motor import Motor

LF_SENSOR = 14
CT_SENSOR = 15
RT_SENSOR = 18

class LineReader():
  def __init__(self):
    self.io = pigpio.pi()
    if not self.io.connected:
      print("Unable to connection to pigpio daemon!")
      sys.exit(0)
    self.io.set_mode(LF_SENSOR, pigpio.INPUT)
    self.io.set_mode(CT_SENSOR, pigpio.INPUT)
    self.io.set_mode(RT_SENSOR, pigpio.INPUT)

    # State of robot
    self.edge = 'L'
    self.lost = True

    # Full Left/Right
    self.full_left = 2.5
    self.full_right = -2.5
    self.find_line = 1

  def read_state_raw(self):
    lf_state = self.io.read(LF_SENSOR)
    ct_state = self.io.read(CT_SENSOR)
    rt_state = self.io.read(RT_SENSOR)
    return (int(lf_state), int(ct_state), int(rt_state))

  def error(self):
    raw_state = self.read_state_raw()
    # If a line is detected, set lost to false
    if 1 in raw_state:
      self.lost = False
    if raw_state == (0,0,0):
      # No line is detected. This could be any of the three cases:
      # At the start
      if self.lost == True:
          # Try to find the line
          # Turn in circle until line is reached
          output = self.find_line
      # It deviates from the line / line ends
      else:
          # Not lost, next step depends on where robot was previously
          if self.edge == 'L':
            # Spin in place to left
            output = self.full_left
          if self.edge == 'R':
            # Spin in place to right
            output = self.full_right
    elif raw_state == (1,0,1):
      # We are at an intersection
      # Decide on a turn
      # Turn left for now
      output = self.full_left
    elif raw_state == (1,1,1):
      # All three sensors are reading a line
      # Keep going straight
      output = 0
    else:
      # These are the normal cases
      # Compute the centroid
      # The dot product allows us to determine the direction of the line
      # The divisor allows us to determine the magnitude of the turn
      # Centroid = (1, 0, -1) * (L, C, R) / (L + C + R)
      # For example, centroid of (L, C, R) = (1, 1, 0) = -0.5
      # (-1, 0, 1) * (1, 1, 0) / (1 + 1 + 0) = -1/2
      # centroid of (L, C, R) = (1, 0, 0) is -1, which is larger than the previous one
      centroid = np.dot((1, 0, -1), raw_state) / sum(np.array(raw_state))
      if centroid > 0:
        self.edge = 'L'
      elif centroid < 0:
        self.edge = 'R'
      output = centroid
    
    return output

  def PID(self):
    error = self.error()
    kp = 50
    steer = kp * error
    return steer


  def shutdown(self):
    self.io.stop()

if __name__ == "__main__":
  line = LineReader()
  motor = Motor()
  try:
    while True:
      steer = line.PID()
      motor.setvel(10, steer)
  except KeyboardInterrupt:
    print("Ending due to keyboard interrupt")
    line.shutdown()
    motor.shutdown()
  except Exception as e:
    print("Ending due to exception: %s" % repr(e))
    line.shutdown()
    motor.shutdown()
