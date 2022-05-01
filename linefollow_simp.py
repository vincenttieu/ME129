#!/usr/bin/env python3

from multiprocessing.sharedctypes import Value
from turtle import right
import pigpio
import sys
import time
import math
from collections import deque
from collections import Counter
from motor import Motor

LF_SENSOR = 14
CT_SENSOR = 15
RT_SENSOR = 18

class LineReaderSimp:
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

    # Full turn = only far left or right sensor reading
    self.full_turn = 100
    # Slight turn = 2 sensors reading
    self.slight_turn = 50

    # Linear Speeds
    self.v_norm = 20

  def read_state_raw(self):
    lf_state = self.io.read(LF_SENSOR)
    ct_state = self.io.read(CT_SENSOR)
    rt_state = self.io.read(RT_SENSOR)
    return (int(lf_state), int(ct_state), int(rt_state))

  def steer(self):
    """ Returns linear, steer
    Respectively, linear speed and angular spin, based on sensor reading

    Line track only, if reads intersection return 0, 0 for both

    Sign Convention: Linear: forward = positive, Angular: positive = CCW / left turn
    """
    raw_state = self.read_state_raw()
    if raw_state in [(1,0,1), (1,1,1), (0,0,0)]:
      # We are at an intersection
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
    return linear, steer

  def shutdown(self):
    self.io.stop()
