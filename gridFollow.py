#!/usr/bin/env python3

from multiprocessing.sharedctypes import Value
from turtle import right
import pigpio
import sys
import time
import math
import random
from collections import deque
from collections import Counter
from motor import Motor
from linefollow_simp import LineReaderSimp

NORTH = 0
WEST  = 1
SOUTH = 2
EAST  = 3
HEADING = {NORTH:'North', WEST:'West', SOUTH:'South',
           EAST:'East', None:'None'} # For printing

# Street status
UNKNOWN    = 'Unknown'
NOSTREET   = 'NoStreet'
UNEXPLORED = 'Unexplored'
CONNECTED  = 'Connected'

# Global Variables: 
intersections   = []   # List of intersections
lastintersecion = None # Last intersection visited

lon     = 0            # Current east/west coordinate
lat     = -1           # Current north/south coordinate
heading = NORTH        # Current heading

class GridFollow:
  def __init__(self):
    self.motor = Motor()
    self.line = LineReaderSimp()

    # Drive Straight
    self.time_after_int = 0.5
    self.v_after_int = 20

    # Turn method
    self.turn_angular_speed = 180

  def driveStraight(self):
    starttime = time.time()
    while True:
      linear, angular = self.line.steer()
      self.motor.setvel(linear, angular)
      # Break out of the loop once we reached intersection
      if linear == 0 and angular == 0:
        break
      # Drive for a bit after the intersection
    self.motor.setvel(self.v_after_int, 0)
    time.sleep(self.time_after_int)
    # Stop the motors
    self.motor.stop()
    # Record the end time
    endtime = time.time()
    # Return the total time elapsed to reach intersection
    return starttime - endtime

  def turn(self, turn_magnitude):
    """ Turns the robot a given amount, with turn_magnitude the following:
    0 for no turn, 1 for left (90 degrees), 2 for backwards (180 degrees), 3 for right (270 degrees)"""
    # Set motor angular speed
    # We are turning at angular speed of 90 deg/s
    turn_magnitude = turn_magnitude % 4
    if turn_magnitude >= 0 and turn_magnitude <= 2:
      self.motor.setspin(self.turn_angular_speed)
      time.sleep(turn_magnitude * 90 / self.turn_angular_speed)
    elif turn_magnitude > 2 and turn_magnitude < 4:
      self.motor.setspin(-(self.turn_angular_speed))
      time.sleep((4 - turn_magnitude) * 90 / self.turn_angular_speed)
    else:
      print('Turn Magnitude Out of Range!')
      raise ValueError
    # Stop the motor
    self.motor.setspin(0)

  def turnTo(self, direction):
    global heading
    direction = direction % 4

    # if aimed direction is the same as current heading, don't change
    if direction - heading == 0:
      return

    # determine turning direction (cw / ccw)
    if (direction - heading)%4 <=2:
      self.motor.setspin(self.turn_angular_speed)
    else:
      self.motor.setspin(-self.turn_angular_speed)
    # turn at least half a second
    time.sleep(0.5)

    # if turn 180, turn at least 1.5 seconds
    if (direction - heading)%4 == 2:
      time.sleep(1)

    # simply turn to the next line
    self.turnToNextLine()
    heading = direction

  def turnToNextLine(self):
    global heading
    start_time = time.time()
    self.motor.setspin(self.turn_angular_speed)
    time.sleep(0.5)
    while True:
      if self.line.read_state_raw() == (0,1,0):
        self.motor.stop()
        break
    used_time = time.time() - start_time
    print(used_time)
    if 0.5 <= used_time < 1.5:
      return 1
    elif 1.5 <= used_time < 2.5:
      return 2
    elif 2.5 <= used_time < 3.5:
      return 3
    else:
      return 4
    
  def drive_sequence(self, turn_sequence):
    """ Drives the robot according to a known sequence
    turn_sequence = List of strings, 'L' is left turn, 'F' is forward, 'R' is right turn
    """
    self.driveStraight()
    turns = {'L':1, 'F':0, 'R':3, 'B':2}
    for turn in turn_sequence:
      self.turn(turns.get(turn, 'F'))
      self.driveStraight()

  def wait(self):
    self.motor.stop()
    time.sleep(0.5)

  def spincheck(self):
    """ Spins robot and checks if there are streets in various positions
    Returns list of bool
    streets = [bool, bool, bool, bool]
    True if there is a street
    streets = [Forward, Left, Back, Right] 
    """

    global heading
    # Initialize all streets to False first
    streets = [False, False, True, False]

    # Detect forward street
    if 1 in self.line.read_state_raw():
      streets[0] = True
    
    total_turns = 0
    while total_turns < 3:
      turn = self.turnToNextLine()
      self.wait()
      heading = (heading + turn) % 4
      streets[heading] = True
      total_turns += turn
    print(total_turns)
    return streets

  def explore(self):
    # Drive straight until next intersection
    self.driveStraight()
    # Fully stop
    self.wait()

    # Calculate new coordinates
    global lon, lat, heading
    lon, lat = shift(lon, lat, heading)
    print(lon, lat)

    # Check if current intersection already exists
    cur = intersection(lon, lat)
    # If not, create a new intersection
    if not cur:
      cur = Intersection(lon, lat)

    current_heading = heading
    # Check and update surrounding streets
    streets = self.spincheck()
    # Store streets in NWSE order
    streets = streets[4-current_heading:] + streets[:4-current_heading]

    for i in range(4):
      if cur.streets[i] == UNKNOWN:
        if streets[i]:
          cur.streets[i] = UNEXPLORED
        else:
          cur.streets[i] = NOSTREET

    # If lastintersection exists, update the relationship between lastintersection and current intersection
    global lastintersecion
    if lastintersecion:
      lastintersecion.neighbors[heading] = cur
      lastintersecion.streets[heading] = CONNECTED
      
       # The backwards direction
      backHeading = (heading+2)%4
      cur.neighbors[backHeading] = lastintersecion
      cur.streets[backHeading] = CONNECTED
      cur.headingToTarget = backHeading

    unexplored_streets = [i for i, x in enumerate(cur.streets) if x == UNEXPLORED]
    if unexplored_streets:
      direction = random.choice(unexplored_streets)
      self.turnTo(direction)
    else:
      connected_streets = [i for i, x in enumerate(cur.streets) if x == CONNECTED]
      direction = random.choice(connected_streets)
      self.turnTo(direction)
      heading = direction
      self.motor.stop()

    # Update lastintersection
    lastintersecion = cur
    print(cur.streets)
  
  def returnToTarget(self):
    global lon, lat, heading
    cur = intersection(lon, lat)
    if not cur:
      print("At the wrong spot!")
      raise ValueError
    
    direction = cur.headingToTarget
    self.turnTo(direction)
    
    self.driveStraight()

    # Calculate new coordinates
    lon, lat = shift(lon, lat, heading)

  def exploreAndReturn(self):
    for _ in range(2):
      self.explore()
    for _ in range(2):
      self.returnToTarget()

  def shutdown(self):
    self.line.shutdown()
    self.motor.shutdown()
    
class Intersection:
  # Initialize - create new intersection at (lon, lat)
  def __init__(self, lon, lat):
    self.lon = lon
    self.lat = lat

    self.neighbors = [None, None, None, None]
    # Status of streets at the intersection, in NWSE directions.
    self.streets = [UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN]
    # Direction to head from this intersection in planned move.
    self.headingToTarget = None

    # You are welcome to implement an arbitrary number of
    # "neighbors" to more directly match a general graph.
    # But the above should be sufficient for a regular grid.

    # Add this to the global list of intersections to make it searchable.
    global intersections
    if intersection(lon, lat) is not None:
        raise Exception("Duplicate intersection at (%2d,%2d)" % (lon,lat))
    intersections.append(self)

   # Print format.
  def __repr__(self):
    return ("(%2d, %2d) N:%s W:%s S:%s E:%s - head %s\n" %
            (self.lon, self.lat, self.streets[0],
              self.streets[1], self.streets[2], self.streets[3],
              HEADING[self.headingToTarget]))

# Helper functions
# New longitude/latitude value after a step in the given heading.
def shift(lon, lat, heading):
  if   heading % 4 == NORTH:
    return (lon, lat+1)
  elif heading % 4 == WEST:
    return (lon-1, lat)
  elif heading % 4 == SOUTH:
    return (lon, lat-1)
  elif heading % 4 == EAST:
    return (lon+1, lat)
  else:
    raise Exception("This can't be")

# Find the intersection
def intersection(lon, lat):
  list = [i for i in intersections if i.lon == lon and i.lat == lat]
  if len(list) == 0:
    return None
  if len(list) > 1:
    raise Exception("Multiple intersections at (%2d,%2d)" % (lon, lat))
  return list[0]

if __name__ == "__main__":
  grid = GridFollow()
  #grid.turn(1)
  #grid.drive_sequence(['L', 'R', 'R', 'R', 'F'])
  #grid.drive_sequence(['R', 'L', 'L', 'L', 'F'])
  try:
    grid.explore()
    grid.explore()
    grid.explore()
    grid.returnToTarget()
    grid.returnToTarget()

    # grid.driveStraight()
    # grid.spincheck()
    # grid.driveStraight()
    # time.sleep(1)
    # streets = grid.spincheck()
    # print(streets)
  except KeyboardInterrupt:
    print("Ending due to keyboard interrupt")
    grid.shutdown()
  except Exception as e:
    print("Ending due to exception: %s" % repr(e))
    grid.shutdown()
