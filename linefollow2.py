#!/usr/bin/env python3

from turtle import right
import pigpio
import sys
import time
import numpy as np

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
    self.prev = (0,0,0)
    self.edge = 'L'
    self.lost = True

    # Full Left/Right
    self.full_left = -2
    self.full_right = 2
    self.find_line = -1

  def read_state_raw(self):
    lf_state = self.io.read(LF_SENSOR)
    ct_state = self.io.read(CT_SENSOR)
    rt_state = self.io.read(RT_SENSOR)
    return (int(lf_state), int(ct_state), int(rt_state))

  def steer(self):
    raw_state = self.read_state_raw()      

    # if raw_state == (0,0,0):
    #   return 100
    # if tape is in the center
    if raw_state in [(0,1,0), (1,1,1)]:
      return 0
    # if tape is to the right
    if raw_state == (0,0,1):
      return -1
    if raw_state == (0,1,1):
      return -0.5
    # if tape is to the left
    if raw_state == (1,0,0):
      return 1
    if raw_state == (1,1,0):
      return 0.5

    return 0
    
  def shutdown(self):
    self.io.stop()
    