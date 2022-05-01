# Global Constants: # Headings
from multiprocessing.sharedctypes import Value
from os import curdir
import random

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

class GridFollow:
  def __init__(self):
    pass

  def driveStraight(self):
    pass

  def checkIntersection(self):
    pass

  def explore(self, prev):
    # Drive straight until next intersection
    self.driveStraight()

    # Calculate new coordinates
    global lon, lat, heading
    lon, lat = shift(lon, lat, heading)

    # Check if current intersection already exists
    cur = intersection(lon, lat)
    # If not, create a new intersection
    if not cur:
      cur = Intersection(lon, lat)

    # Check and update surrounding streets
    streets = self.checkIntersection()
    cur.streets = streets[4-heading:] + streets[:4-heading] # store streets in NWSE order

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
      self.turn(direction - heading)
    else:
      connected_streets = [i for i, x in enumerate(cur.streets) if x == CONNECTED]
      direction = random.choice(connected_streets)
      self.turn(direction - heading)

    # Update lastintersection
    lastintersecion = cur
  
  def returnToTarget(self):
    cur = intersection(lon, lat)
    if not cur:
      print("At the wrong spot!")
      raise ValueError
    
    direction = cur.headingToTarget
    self.turn(direction - heading)
    
    self.driveStraight()

    # Calculate new coordinates
    global lon, lat, heading
    lon, lat = shift(lon, lat, heading)

  def exploreAndReturn(self):
    for _ in range(4):
      self.explore()
    for _ in range(4):
      self.returnto







