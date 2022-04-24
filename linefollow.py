#!/usr/bin/env python3

from multiprocessing.sharedctypes import Value
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
    self.edge = 'C'
    self.lost = True

    # Spin for turns
    # Hard turn = spin in place
    self.hard_turn = 300
    # Full turn = only far left or right sensor reading
    self.full_turn = 100
    # Slight turn = 2 sensors reading
    self.slight_turn = 50

    # Spin for finding line
    self.find_line = 150

    # Linear Speeds
    self.v_find = 15
    self.v_norm = 20
    self.v_turn = 0

    # Start Time
    self.start_time = time.time()

  def read_state_raw(self):
    lf_state = self.io.read(LF_SENSOR)
    ct_state = self.io.read(CT_SENSOR)
    rt_state = self.io.read(RT_SENSOR)
    return (int(lf_state), int(ct_state), int(rt_state))

  def steer(self):
    """ Returns linear, steer
    Respectively, linear speed and angular spin, based on sensor reading

    If robot is still lost, try to find line by spiraling
    If robot has reached end, stop the robot and/or turn around
    If Robot drifted off line, spin in place to left or right depending on previous state
    If getting a line reading, apply slight or no corrections

    Sign Convention: Linear: forward = positive, Angular: positive = CCW / left turn
    """
    raw_state = self.read_state_raw()
    # If a line is detected, set lost to false
    if 1 in raw_state:
      self.lost = False
    if raw_state == (0,0,0):
      # No line is detected. This could be any of the four cases:
      # At the start
      if self.lost == True:
          # Try to find the line
          # Turn in circle until line is reached
          linear = self.v_find
          # Keep on widening the circle as time elapses (reduce angular velocity)
          steer = self.find_line / (1 + 0.2*(time.time() - self.start_time))
      # It deviates from the line / line ends
      else:
          # Not lost, next step depends on where robot was previously
          if self.edge == 'L':
            # Spin in place to left
            steer = self.hard_turn
          elif self.edge == 'R':
            # Spin in place to right
            steer = -self.hard_turn
          elif self.edge == 'C':
            # Previous state was center, Robot reached end of line
            steer = self.hard_turn
          linear = self.v_turn
    elif raw_state == (1,0,1):
      # We are at an intersection
      # Decide on a turn
      # Turn left for now
      steer = self.full_turn
      linear = self.v_turn
    else:
      linear = self.v_norm
      # These are the normal cases
      # If the robot is at the center, or close to the center, set edge to 'C'
      if raw_state in [(0,1,0), (1,1,1), (0,1,1), (1,1,0)]:
        self.edge = 'C'
      # If the tape is in the center or head on, keep going straight
      if raw_state in [(0,1,0), (1,1,1)]:
        steer = 0
      # if tape is to the far right, set edge to 'R' and turn right
      if raw_state == (0,0,1):
        self.edge = 'R'
        steer = -self.full_turn
      # If tape is slightly to the right, turn a little right
      if raw_state == (0,1,1):
        steer = -self.slight_turn
      # if tape is to the left
      if raw_state == (1,0,0):
        self.edge = 'L'
        steer = self.full_turn
      if raw_state == (1,1,0):
        steer = self.slight_turn
      
      

      """ Centroid Implementation
      # Compute the centroid
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
      """
    
    return linear, steer


  def shutdown(self):
    self.io.stop()

if __name__ == "__main__":
  line = LineReader()
  motor = Motor()
  try:
    while True:
      linear, angular = line.steer()
      motor.setvel(linear, angular)
  except KeyboardInterrupt:
    print("Ending due to keyboard interrupt")
    line.shutdown()
    motor.shutdown()
  except Exception as e:
    print("Ending due to exception: %s" % repr(e))
    line.shutdown()
    motor.shutdown()
