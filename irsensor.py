import pigpio
import sys
import threading

LF_SENSOR = 14
CT_SENSOR = 15
RT_SENSOR = 18

class IRSensor:

  def __init__(self, io=None):
    if io:
      self.io = io
      self.shutdown_flag = False
    else:
      self.io = pigpio.pi()
      self.shutdown_flag = True
    if not self.io.connected:
      print("Unable to connection to pigpio daemon!")
      sys.exit(0)
    self.io.set_mode(LF_SENSOR, pigpio.INPUT)
    self.io.set_mode(CT_SENSOR, pigpio.INPUT)
    self.io.set_mode(RT_SENSOR, pigpio.INPUT)
  
  def read_state_raw(self):
    lf_state = self.io.read(LF_SENSOR)
    ct_state = self.io.read(CT_SENSOR)
    rt_state = self.io.read(RT_SENSOR)
    return (int(lf_state), int(ct_state), int(rt_state))

  def shutdown(self):
    if self.shutdown_flag:
      self.io.stop()
