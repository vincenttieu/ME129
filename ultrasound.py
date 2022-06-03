import pigpio
import sys
import time
import threading

US1_TRIGGER = 13
US2_TRIGGER = 19
US3_TRIGGER = 26

US_TRIGGERS = [US1_TRIGGER, US2_TRIGGER, US3_TRIGGER]

US1_ECHO = 16
US2_ECHO = 20
US3_ECHO = 21

US_ECHOS = [US1_ECHO, US2_ECHO, US3_ECHO]

US1_RISING_TIME = 0
US2_RISING_TIME = 0
US3_RISING_TIME = 0

GPIO_INDEX = {US1_ECHO: 0, US2_ECHO: 1, US3_ECHO: 2}

TRIGGER_DURATION = 20e-6

HIGH = 1
LOW = 0

class Ultrasound:
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
    self.io.set_mode(US1_TRIGGER, pigpio.OUTPUT)
    self.io.set_mode(US2_TRIGGER, pigpio.OUTPUT)
    self.io.set_mode(US3_TRIGGER, pigpio.OUTPUT)
    self.io.set_mode(US1_ECHO, pigpio.INPUT)
    self.io.set_mode(US2_ECHO, pigpio.INPUT)
    self.io.set_mode(US3_ECHO, pigpio.INPUT)

    
    self.io.set_pull_up_down(US1_ECHO, pigpio.PUD_DOWN)
    self.io.set_pull_up_down(US2_ECHO, pigpio.PUD_DOWN)
    self.io.set_pull_up_down(US3_ECHO, pigpio.PUD_DOWN)

    # Ultrasound variables
    self.distance = [0,0,0]
    self.rising_edge = [0,0,0]

    # Set callback functions
    self.cbrise0 = self.io.callback(US1_ECHO, pigpio.RISING_EDGE, self.rising)
    self.cbrise1 = self.io.callback(US2_ECHO, pigpio.RISING_EDGE, self.rising)
    self.cbrise2 = self.io.callback(US3_ECHO, pigpio.RISING_EDGE, self.rising)
    self.cbfall0 = self.io.callback(US1_ECHO, pigpio.FALLING_EDGE, self.falling)
    self.cbfall1 = self.io.callback(US2_ECHO, pigpio.FALLING_EDGE, self.falling)
    self.cbfall2 = self.io.callback(US3_ECHO, pigpio.FALLING_EDGE, self.falling)

    # Set timer to read ultrasound sensors every 100 ms.
    self.thread = threading.Thread(target=self.runcontinual)
    self.thread.start()

  # Pull trigger high for 10 us
  def trigger(self):
    for i in range(3):
      self.io.write(US_TRIGGERS[i], HIGH)

    time.sleep(TRIGGER_DURATION)

    for i in range(3):
      self.io.write(US_TRIGGERS[i], LOW)

  # read ultrasound sensors
  def stopcontinual(self):
    self.stopflag = True

  def runcontinual(self):
    self.stopflag = False
    while not self.stopflag:
      self.trigger()
      time.sleep(0.1)

  def rising(self, gpio, level, tick):
    # store the rising edge timing
    index = GPIO_INDEX[gpio]
    self.rising_edge[index] = tick

  def falling(self, gpio, level, tick):
    index = GPIO_INDEX[gpio]
    t = float(tick - self.rising_edge[index]) / 1e6
    distance = 343 * t/2 * 100 # in cm
    self.distance[index] = distance

  def shutdown(self):
    self.stopcontinual()
    self.thread.join()

    self.cbrise0.cancel()
    self.cbrise1.cancel()
    self.cbrise2.cancel()
    self.cbfall0.cancel()
    self.cbfall1.cancel()
    self.cbfall2.cancel()
    if self.shutdown_flag:
      self.io.stop()

if __name__ == "__main__":
  try:
    US = Ultrasound()
    while True:
      print("Left: {:5.2f}, Mid: {:5.2f}, Right: {:5.2f}".format(US.distance[0], US.distance[1], US.distance[2]))
      time.sleep(0.1)
  except KeyboardInterrupt:
    print("Ending due to keyboard interrupt")
    US.shutdown()
  except Exception as e:
    print("Ending due to exception: %s" % repr(e))
    US.shutdown()
