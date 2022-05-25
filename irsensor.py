import pigpio
import sys
import threading

LF_SENSOR = 14
CT_SENSOR = 15
RT_SENSOR = 18

class IRSensor:

  def __init__(self):
    self.io = pigpio.pi()
    if not self.io.connected:
      print("Unable to connection to pigpio daemon!")
      sys.exit(0)
    self.io.set_mode(LF_SENSOR, pigpio.INPUT)
    self.io.set_mode(CT_SENSOR, pigpio.INPUT)
    self.io.set_mode(RT_SENSOR, pigpio.INPUT)
    
    self.thread = threading.Thread(target=self.readIR_loop)
    self.thread.start()
  
  def read_state_raw(self):
    lf_state = self.io.read(LF_SENSOR)
    ct_state = self.io.read(CT_SENSOR)
    rt_state = self.io.read(RT_SENSOR)
    return (int(lf_state), int(ct_state), int(rt_state))

  def readIR_stop(self):
    self.stopflag = True

  def readIR_loop(self):
    self.stopflag = False
    while not self.stopflag:
      self.IRreading = self.read_state_raw()

  def shutdown(self):
    self.readIR_stop()
    self.thread.join()
    self.io.stop()
