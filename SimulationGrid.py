#!/usr/bin/env python3


import sys
import time
import math
import random
from collections import deque
from collections import Counter

import matplotlib.pyplot as plt
import numpy as np

plt.ion()

NORTH = 0
WEST = 1
SOUTH = 2
EAST = 3
HEADING = {NORTH: 'North', WEST: 'West', SOUTH: 'South',
           EAST: 'East', None: 'None'}  # For printing

# Street status
UNKNOWN = 'Unknown'
NOSTREET = 'NoStreet'
UNEXPLORED = 'Unexplored'
CONNECTED = 'Connected'

# Global Variables:
intersections = []  # List of intersections
lastintersecion = None  # Last intersection visited

lon = 0  # Current east/west coordinate
lat = -1  # Current north/south coordinate
heading = NORTH  # Current heading

######################################################################################
# SIMULATION STARTS HERE
# All this does is create a simulated grid with connected/not connected intersections
# Only the driving, turning and spincheck functions were changed, such that instead of activating the motors
# and sensors, they proceed along the simulated grid.
# The explore and intersection algorithms are the same.
# This essentially simulates how the robot would behave if there were no hardware issues and actually behaved correctly
# while calling the driving, turning, and spincheck functions

fig, ax = plt.subplots()

# Simulation Variables

# Simulator is list of lists, where simulator[lon][lat] corresponds to a intersection
# simulator[lon][lat] = [bool, bool, bool, bool] corresponding to whether there is a connection
# at [North, West, South, East]

simulator = [[[True, False, False, True], [True, False, True, True], [False, False, True, True]],
             [[True, True, False, True],  [True, True, True, True],  [False, True, True, True]],
             [[True, True, False, False], [True, True, True, False], [False, True, True, False]]]

simulator = [[[True, False, False, True], [False, False, True, True], []],
             [[True, True, False, True],  [True, True, True, True],  [False, False, True, True]],
             [[True, True, False, False], [True, True, True, False], [False, True, True, False]]]

# Simulator longitude, latitude, and heading
# This simulates the movement of the actual robot
# These are updated in the drivestraight and turn functions as opposed to lon, lat, and heading
# which are updated in the grid search algorithm

simulator_lon = 0
simulator_lat = -1
simulator_heading = NORTH



def pathplot(loc, conn, fig, ax):
    """ Plot a single intersection at loc=(lon, lat) with connections [bool,bool,bool,bool]
    Takes in fig, ax from plt.subplots()
    """
    x, y = loc
    ax.plot(x, y, 'b', marker="o", markersize=20)
    for i, state in enumerate(conn):
        if state == NOSTREET:
            fmt = 'r--'
        elif state == UNKNOWN:
            fmt = 'r-'
        elif state == UNEXPLORED:
            fmt = 'y-'
        else:
            fmt = 'g'
        ax.plot([x, x-0.25*np.sin(i*np.pi/2)], [y, y+0.25*np.cos(i*np.pi/2)], fmt)

def plotall(fig, ax):
    """ Iterate over list of intersections and plots all of them """
    global lon, lat
    for intersect in intersections:
        pathplot((intersect.lon, intersect.lat), intersect.streets, fig, ax)

    ax.plot(lon, lat, 'r', marker="o", markersize=15)
    ax.set_title('Intersections found by Robot')
    fig.canvas.draw()
    fig.canvas.flush_events()

def plotsim():
    """ Plot the intersections of the actual simulation grid """
    global simulator
    fig, ax = plt.subplots()
    for lon in range(len(simulator)):
        for lat in range(max([len(x) for x in simulator])):
            try:
                streets = simulator[lon][lat]
                if streets:
                    conn = [UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN]
                    for i, state in enumerate(streets):
                        if state:
                            conn[i] = CONNECTED
                        else:
                            conn[i] = NOSTREET
                    pathplot((lon, lat), conn, fig, ax)
            except IndexError:
                pass
    ax.set_title('Actual Simulator Grid')

    fig.canvas.draw()
    fig.canvas.flush_events()


class GridFollow:
    def __init__(self):

        # Drive Straight
        self.time_after_int = 0.5
        self.v_after_int = 20

        # Turn method
        self.turn_angular_speed = 90

    def driveStraight(self):
        """ Simulate driving straight. All it does is increment the simulation lat, lon according
        to the simulation heading
        """
        global simulator_heading, simulator_lat, simulator_lon
        print('Driving in direction: ', HEADING[simulator_heading%4])

        if simulator_heading == NORTH:
            simulator_lat += 1
        elif simulator_heading == WEST:
            simulator_lon -= 1
        elif simulator_heading == SOUTH:
            simulator_lat -= 1
        elif simulator_heading == EAST:
            simulator_lon += 1
        return

    def turn(self, turn_magnitude):
        """ Turns the robot a given amount, with turn_magnitude the following:
        0 for no turn, 1 for left (90 degrees), 2 for backwards (180 degrees), 3 for right (270 degrees)"""
        # Set motor angular speed
        # We are turning at angular speed of 90 deg/s
        turn_magnitude = turn_magnitude % 4
        print('Turning', turn_magnitude)
        return

    def turnTo(self, direction):
        """ Simulate turning to a given heading """
        global heading, simulator_heading
        print("Current: ", HEADING[heading%4], " ,aimed: ", HEADING[direction%4])

        heading = direction
        simulator_heading = direction

    def drive_sequence(self, turn_sequence):
        """ Drives the robot according to a known sequence
        turn_sequence = List of strings, 'L' is left turn, 'F' is forward, 'R' is right turn
        """
        self.driveStraight()
        turns = {'L': 1, 'F': 0, 'R': 3, 'B': 2}
        for turn in turn_sequence:
            self.turn(turns.get(turn, 'F'))
            self.driveStraight()

    def spincheck(self):
        """
        Simulate performing a spin check
        All this does is poll the simulator grid at the simulated lon lat and return
        the list of connections shifted to the current simulator heading

        Spins robot and checks if there are streets in various positions
        Returns list of bool
        streets = [bool, bool, bool, bool]
        True if there is a street
        streets = [Forward, Left, Back, Right]
        """
        print('Checking Intersection')
        global simulator_lon, simulator_lat, simulator, simulator_heading
        streets = simulator[simulator_lon][simulator_lat]
        streets = streets[simulator_heading:] + streets[:simulator_heading]

        return streets

####################################################################################
# END SIMULATOR SECTION
# BEGIN NORMAL CODE

    def explore(self):
        # Drive straight until next intersection
        self.driveStraight()
        # Wait 0.5 second to fully stop
        time.sleep(0.5)

        # Calculate new coordinates
        global lon, lat, heading
        lon, lat = shift(lon, lat, heading)

        # Check if current intersection already exists
        cur = intersection(lon, lat)
        # If not, create a new intersection
        if not cur:
            cur = Intersection(lon, lat)

        # Check and update surrounding streets
        streets = self.spincheck()
        # Store streets in NWSE order
        streets = streets[4 - heading:] + streets[:4 - heading]

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
            backHeading = (heading + 2) % 4
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


        # Update lastintersection
        lastintersecion = cur

        plotall(fig, ax)
        print(cur.streets)

    def dijkstra(self, target_lon, target_lat):
        print('Going to target', target_lon, target_lat)
        global lon, lat
        # Clear all headingToTarget directions stored in all intersections
        for i in intersections:
            i.headingToTarget = None
        # Get the target intersection
        target = intersection(target_lon, target_lat)
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
        while (lon, lat) != (target_lon, target_lat):
            self.returnToTarget()

    def returnToTarget(self):
        time.sleep(0.5)
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

        plotall(fig, ax)

    def exploreAndReturn(self):
        for _ in range(2):
            self.explore()
        for _ in range(2):
            self.returnToTarget()

    def shutdown(self):
        pass


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
            raise Exception("Duplicate intersection at (%2d,%2d)" % (lon, lat))
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
    if heading % 4 == NORTH:
        return (lon, lat + 1)
    elif heading % 4 == WEST:
        return (lon - 1, lat)
    elif heading % 4 == SOUTH:
        return (lon, lat - 1)
    elif heading % 4 == EAST:
        return (lon + 1, lat)
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


def unexplored():
    return [i for i in intersections
    if any([s==UNEXPLORED for s in i.streets])]



if __name__ == "__main__":

    grid = GridFollow()
    # grid.turn(1)
    # grid.drive_sequence(['L', 'R', 'R', 'R', 'F'])
    # grid.drive_sequence(['R', 'L', 'L', 'L', 'F'])
    try:

        while not searchcomplete():
            grid.explore()

        (trav_x, trav_y) = (2,2)
        print('Using Dijkstra\'s Algorithm to travel to', trav_x, trav_y)
        grid.dijkstra(trav_x, trav_y)

        plotsim()

    except KeyboardInterrupt:
        print("Ending due to keyboard interrupt")
        grid.shutdown()
    except Exception as e:
        print("Ending due to exception: %s" % repr(e))
        grid.shutdown()
