#!/usr/bin/env python3

import pigpio
import sys
import time
import random
import threading
from motor import Motor
from linefollow_simp import LineReaderSimp
from angle import angle
import pickle

NORTH = 0
WEST  = 1
SOUTH = 2
EAST  = 3
STOP  = 'STOP'
HEADING = {NORTH:'North', WEST:'West', SOUTH:'South',
           EAST:'East', None:'None', STOP:'STOP'} # For printing

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
    # Initialize motor, line following, and gyro
    self.motor = Motor()
    self.line = LineReaderSimp()
    self.gyro = angle()

    # Calibrate the following
    self.time_after_int = 0.5 # pwm
    self.v_after_int = 20 # ms
    self.turn_speed = 0.7 # pwm
    self.turn_speed_slow = 0.4 # pwm

    # driving thread flags
    self.driving_stopflag = False
    self.driving_pause = False

    self.target = None


  def driveStraight(self):
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

  def turnTo(self, direction):
    global heading
    direction = direction % 4
    
    turn = (direction - heading) % 4
    if turn == 0:
      return
    elif turn == 1:
      self.turnToNextLine()
    elif turn == 2:
      turned = 0
      while turned < 2:
        turned += self.turnToNextLine()
    else:
      self.turnToNextLine(dir=-1)

    heading = direction

  def turnToNextLine(self, dir=1):
    # turn to the next line
    # direction =  1 -> counter-clockwise
    # direction = -1 -> clockwise
    global heading

    self.gyro.reset() # resets the gyro to prevent error from accumulating
    angle = 0

    self.motor.set(-dir*self.turn_speed, dir*self.turn_speed)
    while True:
      angle = abs(self.gyro.angle)
      reading = self.line.read_state_raw()
      #print(reading)
      if angle > 20:
        if reading == (0,1,0):
          self.motor.stop()
          break
        elif reading in [(1,1,0),(0,1,1)]:
          self.motor.set(-dir*self.turn_speed_slow, dir*self.turn_speed_slow)
    
    turned = (angle+45) // 90
    if turned < 1 or turned > 4:
      print("Turned ", turned)
      raise ValueError()
    return int((angle+45) // 90)

  def wait(self):
    self.motor.stop()
    time.sleep(0.5)

  def spinCheck(self):
    """ Spins robot and checks if there are streets in various positions
    Returns list of bool
    streets = [bool, bool, bool, bool]
    True if there is a street
    streets = [Forward, Left, Back, Right] 
    """

    global heading
    # Initialize all streets except for the back street to False first
    streets = [False, False, True, False]

    # Detect forward street
    if 1 in self.line.read_state_raw():
      streets[0] = True
    
    total_turned = 0
    # Check surroundings until turned at least 270 degrees
    while total_turned < 3:
      # turn to next line
      turned = self.turnToNextLine()
      self.wait()
      # update heading
      heading = (heading + turned) % 4
      # update total_turned
      total_turned += turned
      streets[total_turned%4] = True

    return streets
      
  def explore(self):
    # Drive straight until next intersection
    self.driveStraight()
    # Fully stop
    self.wait()

    # Calculate new coordinates
    global lon, lat, heading
    lon, lat = shift(lon, lat, heading)
    # print("lon: {lon}, lat: {lat}, heading: {heading}".format(lon=lon, lat=lat, heading=heading))

    # Check if current intersection already exists
    cur = intersection(lon, lat)

    # Store current heading, since spinCheck() may change the heading
    current_heading = heading

    # If not, create a new intersection and check surrondings
    if not cur:
      cur = Intersection(lon, lat)

      # Check and update surrounding streets
      streets = self.spinCheck()
      # Store streets in NWSE order
      streets = streets[4-current_heading:] + streets[:4-current_heading]

      # Update surrounding streets
      for i in range(4):
        if cur.streets[i] == UNKNOWN:
          if streets[i]:
            neighborLon, neighborLat = shift(lon, lat, i)
            neighbor = intersection(neighborLon, neighborLat)
            if neighbor:
              cur.streets[i] = CONNECTED
              neighbor.streets[(i+2)%4] = CONNECTED
            else:
              cur.streets[i] = UNEXPLORED
          else:
            cur.streets[i] = NOSTREET

    # If lastintersection exists, update the relationship between lastintersection and current intersection
    global lastintersecion
    if lastintersecion:
      lastintersecion.neighbors[current_heading] = cur
      lastintersecion.streets[current_heading] = CONNECTED
      
      # The backwards direction
      backHeading = (current_heading+2)%4
      cur.neighbors[backHeading] = lastintersecion
      cur.streets[backHeading] = CONNECTED
      cur.headingToTarget = backHeading

    # If there are unexplored streets, randomly go to an unexplored street
    unexplored_streets = [i for i, x in enumerate(cur.streets) if x == UNEXPLORED]
    if unexplored_streets:
      direction = random.choice(unexplored_streets)
      self.turnTo(direction)
    # Else, randomly go to a connected street
    else:
      # If there are still unexplored intersections, use Dijkstra's algorithm to deterministically explore
      # Set the headingToTargets to the nearest unexplored intersection
      unexplored_intersections = unexplored()
      if unexplored_intersections:
        # print("EXPLORED: ", unexplored_intersections[0].lon, unexplored_intersections[0].lat, unexplored_intersections[0].streets)
        cur = self.goToTarget(unexplored_intersections[0])
        # print(cur.streets)
        for i in range(4):
          if cur.streets[i] == UNEXPLORED:
            direction = i
            break
        # self.dijkstra(unexplored_intersections[0].lon, unexplored_intersections[0].lat)
        # direction = cur.headingToTarget
      # Otherwise, the whole map is already explored
      else:
        print("Finished Exploring the Map!")
        return
      self.turnTo(direction)
      heading = direction
      self.motor.stop()

    # Update lastintersection
    lastintersecion = cur
  
  def dijkstra(self, target_lon, target_lat):
    """ Set headings of intersections according to desired target intersection """
    print('Set Headings to target', target_lon, target_lat)
    global lon, lat
    # Clear all headingToTarget directions stored in all intersections
    for i in intersections:
      i.headingToTarget = None
    # Get the target intersection
    target = intersection(target_lon, target_lat)
    print(target)
    # Initialize the "to be processed" intersections list with the target
    inters_to_process = [target]
    # Iterating lon, lat to go through the intersections. Start at target intersection
    curlon, curlat = target_lon, target_lat
    # Loop through until we get back to the original start intersection
    while (curlon, curlat) != (lon, lat):
      # Pop the first element of the to-be-processed list as temporary target
      tempTarget = inters_to_process.pop(0)
      # Get the coordinates of this temporary target
      curlon, curlat = tempTarget.lon, tempTarget.lat
      # Get the list of directions to intersections connected to temporary target
      tempConnectedStreets = [i for i, x in enumerate(tempTarget.streets) if x == CONNECTED]
      # For each connected intersection of the temporary target:
      for dir in tempConnectedStreets:
        # Get the connected intersection object and coordinates
        conn_lon, conn_lat = shift(curlon, curlat, dir)
        tempConnected = intersection(conn_lon, conn_lat)
        if tempConnected.headingToTarget is None:
          # If the connected intersection does not have a headingToTarget:
          # Set the heading so it points back to the temporary target
          # Append the connected intersection to to-be-processed list
          tempConnected.headingToTarget = (dir + 2) % 4
          inters_to_process.append(tempConnected)
    target.headingToTarget = STOP

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

  def goToTarget(self, target):
    target_lon = target.lon
    target_lat = target.lat
    cur = intersection(lon, lat)
    self.dijkstra(target_lon, target_lat)
    while cur.headingToTarget is not STOP:
      self.returnToTarget()
      cur = intersection(lon, lat)
    return cur

  def exploreAndReturn(self):
    for _ in range(5):
      self.explore()
    for _ in range(4):
      self.returnToTarget()
  
  def exploreWholeMap(self):
    self.thread = threading.Thread(target=self.driving_loop)
    self.thread.start()

  def driving_stop(self):
    self.driving_stopflag = True

  def driving_goto(self, target):
    self.target = target
  
  def driving_loop(self):
    self.driving_stopflag = False
    # if the thread is not stopped or the map isn't fully explored
    while (not self.driving_stopflag):# and (not searchcomplete()):
      # Pause at this intersection, if requested
      if not self.driving_pause:
        self.explore()
      if self.target:
        self.goToTarget(self.target)
        self.target = None

  def shutdown(self):
    self.driving_stop()
    self.thread.join()
    self.motor.stop()

    self.line.shutdown()
    self.motor.shutdown()
    self.gyro.shutdown()
    
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

# Return list of unexplored intersections, in the order of nearest to farthest
def unexplored():
  unexplored_intersects = [i for i in intersections
  if any([s==UNEXPLORED for s in i.streets])]
  # Sort list of unexplored intersections from nearest to farthest
  unexplored_intersects = sorted(unexplored_intersects, key=lambda intersect: (intersect.lon - lon)**2 + (intersect.lat - lat)**2)

  return unexplored_intersects

# Return true if all intersections have been mapped, otherwise false
def searchcomplete():
  global intersections
  if intersections:
    complete = True
    for intersection in intersections:
      if UNKNOWN in intersection.streets or UNEXPLORED in intersection.streets:
        complete = False
  else:
    complete = False
  return complete

def userInput(grid):
  global intersections, lastintersecion, lon, lat, heading
  # Grab a command
  command = input("Command ? ")
  command = command.strip()
  # Compare against possible commands.
  if (command == 'pause'):
    print("Pausing at the next intersection")
    grid.driving_pause = True
  elif (command == 'explore'):
    print("Exploring without a target")
    grid.driving_pause = False
  elif (command[:4] == 'goto'):
    grid.driving_pause = True
    coord = command.split(" ")
    target = intersection(int(coord[1]), int(coord[2]))
    if not target:
      print("Not a valid position!")
    else:
      grid.driving_goto(target)
  elif (command == 'save'):
    filename = input('Input the name you want to save the map as: ')
    print("Saving the map...")
    
    with open(filename+'.pickle', 'wb') as file:
      pickle.dump(intersections, file)
  
  elif (command == 'load'):
    filename = input('Type in the name of the map you want to load: ')
    print('Loading the map')
    try:
      with open(filename+'.pickle', 'rb') as file:
        intersections = pickle.load(file)
      lastintersecion = None
      cur_loc = input("Input the current location on map: ")
      cur_coord = cur_loc.split(" ")
      lon = int(cur_coord[0])
      lat = int(cur_coord[1])
      print(lon, lat)
      cur_heading = input("Input the current heading (0 to 3)")
      heading = int(cur_heading)
    except FileNotFoundError:
      print("File {}.pickle was not found.".format(filename))
    
  
  elif (command == 'home'):
    print("Restarting the robot at the home position")
    lon = 0
    lat = 0

  elif (command == 'print'):
    print(intersections)
  elif (command == 'quit'):
    print("Quitting...")
    return True


  else:
    print("Unknown command '%s'" % command)
  return False

if __name__ == "__main__":
  grid = GridFollow()
  grid.exploreWholeMap()
  try:
    while True:
      quit = userInput(grid)
      if quit:
        break
  except KeyboardInterrupt:
    print("Ending due to keyboard interrupt")
  except Exception as e:
    print("Ending due to exception: %s" % repr(e))

  grid.shutdown()