import time
import math
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
import threading


class angle:
  def __init__(self):
    self.mpu = MPU9250(
                address_ak=AK8963_ADDRESS, 
                address_mpu_master=MPU9050_ADDRESS_68, # In 0x68 Address
                address_mpu_slave=None, 
                bus=1,
                gfs=GFS_1000, 
                afs=AFS_8G, 
                mfs=AK8963_BIT_16, 
                mode=AK8963_MODE_C100HZ)
    
    self.mpu.configure()
    self.offset = 0
    self.calibrate()
    self.last_read = time.time()
    self.angle = 0

  def calibrate(self, samples=100):
    counter = 0
    sum_of_angles = 0
    while counter < samples:
      sum_of_angles += self.mpu.readGyroscopeMaster()[2]
      counter += 1
    self.offset = sum_of_angles / samples
    
  def readAngle(self):
    ang_vel = self.mpu.readGyroscopeMaster()[2] - self.offset
    cur_time = time.time()
    self.angle += ang_vel * (cur_time - self.last_read)
    self.last_read = cur_time
    return self.angle

  def reset(self):
    self.resetAngle(0)
    self.resetTime()
  
  def resetAngle(self, cur_angle):
    self.angle = cur_angle
  
  def resetTime(self):
    self.last_read = time.time()

if __name__ == "__main__": 
  ang = angle()
  while True:
    try:
      print(ang.readAngle())
    except KeyboardInterrupt:
      break
    except Exception as e:
      break