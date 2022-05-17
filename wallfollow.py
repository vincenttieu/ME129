import pigpio
import sys
import time
import threading
import random
from ultrasound import Ultrasound
from motor import Motor

FWD_VEL = 0.7

class WallFollow:
  def __init__(self):
    self.motor = Motor()
    self.US = Ultrasound()
    time.sleep(0.5)

  def driveToObstacle(self, d=20):
    while self.US.distance[1] > d:
      print(self.US.distance[1])
      self.motor.set(FWD_VEL, FWD_VEL)
    self.motor.stop()
    print('Obstacle Reached')
  
  def herding(self, d=20):
    while True:
      obstacles = [self.US.distance[0]<d, self.US.distance[1]<d, self.US.distance[2]<d]
      print(obstacles)
      # If something is in the way in front and the sides, go backwards straight
      if obstacles in [[True, True, True], [False, True, False]]:
        self.motor.set(-FWD_VEL, -FWD_VEL)
      # If something is on the sides or nothing anywhere, go straight
      elif obstacles in [[True, False, True], [False, False, False]] :
        self.motor.set(FWD_VEL, FWD_VEL)
      # If something is to the left, turn right
      elif obstacles == [True, False, False]:
        self.motor.set(FWD_VEL+0.2, FWD_VEL-0.2)
      # If something is to the right, turn left
      elif obstacles == [False, False, True]:
        self.motor.set(FWD_VEL-0.2, FWD_VEL+0.2)
      # If something is to the left and in front, backwards 
      elif obstacles == [True, True, False]:
          self.motor.set(FWD_VEL, -FWD_VEL)
      # If something is to the right and in front, backwards 
      elif obstacles == [False, True, True]:
          self.motor.set(-FWD_VEL, FWD_VEL)
      time.sleep(0.5)

  def move(self, u):
    PWM_left = max(0.5, min(0.9, 0.7 - u))
    PWM_right = max(0.5, min(0.9, 0.7 + u))
    self.motor.set(PWM_left, PWM_right)
  
  def wallFollow(self, d=20, k=0.05):
    while True:
      dist = self.US.distance[2]
      error = dist-d
      u = -k*error
      self.move(u)
      print(dist)
      if self.US.distance[1] < d:
        break
    self.motor.stop()
    print('Obstacle')
    


  def shutdown(self):
    self.motor.shutdown()
    self.US.shutdown()

if __name__ == "__main__":
  try:
    wf = WallFollow()
    #wf.herding()
    wf.wallFollow()
  except KeyboardInterrupt:
    print("Ending due to keyboard interrupt")
    wf.shutdown()
  except Exception as e:
    print("Ending due to exception: %s" % repr(e))
    wf.shutdown()



        
      

