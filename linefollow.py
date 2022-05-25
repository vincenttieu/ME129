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
from irsensor import IRSensor


class LineReader():
  def __init__(self):
    self.irsensor = IRSensor()

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
    self.v_find = 20
    self.v_norm = 20
    self.v_turn = 0

    # On-track Time
    self.last_seen = time.time()

    # Queue to store the states
    self.queue = deque()

  def read_state_raw(self):
    return self.irsensor.read_state_raw()

  def steer(self):
    """ Returns linear, steer
    Respectively, linear speed and angular spin, based on sensor reading

    If robot is still lost, try to find line by spiraling
    If robot has reached end, stop the robot and/or turn around
    If Robot drifted off line, spin in place to left or right depending on previous state
    If getting a line reading, apply slight or no corrections

    Sign Convention: Linear: forward = positive, Angular: positive = CCW / left turn
    """
    raw_state = self.irsensor.read_state_raw()
    # If a line is detected, set lost to false
    if 1 in raw_state:
      self.lost = False
      self.last_seen = time.time()
    if raw_state == (0,0,0):
      # If the robot has not detected anything for over 5 seconds,
      # set lost to be true
      if time.time() - self.last_seen > 5:
        self.lost = True
      # No line is detected. This could be any of the four cases:
      # At the start
      if self.lost == True:
          # Try to find the line
          # Turn in circle until line is reached
          linear = self.v_find
          # Keep on widening the circle as time elapses (reduce angular velocity)
          steer = self.find_line / (1 + 2*(time.time() - self.last_seen))
      # It deviates from the line / line ends
      else:
          # Not lost, next step depends on where robot was previously
          if self.readState() == 'L':
            # Spin in place to left
            steer = self.hard_turn
          elif self.readState() == 'R':
            # Spin in place to right
            steer = -self.hard_turn
          elif self.readState() == 'C':
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
      if raw_state in [(0,1,0), (1,1,1)]:
        self.storeState('C')
      # If the tape is in the center or head on, keep going straight
      if raw_state in [(0,1,0), (1,1,1)]:
        steer = 0
      # if tape is to the far right, set edge to 'R' and turn right
      if raw_state == (0,0,1):
        self.storeState('R')
        steer = -self.full_turn
      # If tape is slightly to the right, turn a little right
      if raw_state == (0,1,1):
        self.storeState('R')
        steer = -self.slight_turn
      # if tape is to the left
      if raw_state == (1,0,0):
        self.storeState('L')
        steer = self.full_turn
      if raw_state == (1,1,0):
        self.storeState('L')
        steer = self.slight_turn
    return linear, steer

  # store the most recent 500 states
  def storeState(self, state):
    self.queue.append(state)
    if len(self.queue) > 500:
      self.queue.popleft()

  # get the most frequent state in queue
  def readState(self):
    return max(set(self.queue), key=self.queue.count)

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
