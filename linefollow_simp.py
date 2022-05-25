#!/usr/bin/env python3

import pigpio
import sys
import time
import math
from irsensor import IRSensor


class LineReaderSimp:
  def __init__(self):

    self.irsensor = IRSensor()

    # State of robot
    self.last_state = (0,0,0)

    # Full turn = only far left or right sensor reading
    self.full_turn = 100
    # Slight turn = 2 sensors reading
    self.slight_turn = 50

    # Linear Speeds
    self.v_norm = 20

  def read_state_raw(self):
    """ Access sensor reading from IR Sensor layer """
    return self.irsensor.IRreading

  def steer(self):
    """ Returns linear, steer
    Respectively, linear speed and angular spin, based on sensor reading

    Line track only, if reads intersection return 0, 0 for both

    Sign Convention: Linear: forward = positive, Angular: positive = CCW / left turn
    """
    raw_state = self.irsensor.IRreading
    if raw_state in [(1,0,1), (1,1,1)]:
      # We are at an intersection
      linear = 0
      steer = 0
    elif raw_state == (0,0,0):
      if self.last_state == (0,0,1):
        linear =self.v_norm
        steer = -self.full_turn
      elif self.last_state == (1,0,0):
        linear = self.v_norm
        steer = self.full_turn
      else:
        linear = 0
        steer = 0
    else:
      # Update time
      self.intersection_time = time.time()
      linear = self.v_norm
      # These are the normal cases
      # If the tape is in the center or head on, keep going straight
      if raw_state == (0,1,0):
        steer = 0
      # if tape is to the far right, set edge to 'R' and turn right
      if raw_state == (0,0,1):
        steer = -self.full_turn
      # If tape is slightly to the right, turn a little right
      if raw_state == (0,1,1):
        steer = -self.slight_turn
      # if tape is to the left
      if raw_state == (1,0,0):
        steer = self.full_turn
      # If tape is slightly to the left
      if raw_state == (1,1,0):
        steer = self.slight_turn 
    
    if 1 in raw_state:
      self.last_state = raw_state
    return linear, steer

  def shutdown(self):
    self.irsensor.shutdown()
