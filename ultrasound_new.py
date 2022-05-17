import pigpio
import sys
import time
import threading
import random

US1_TRIGGER = 13
US2_TRIGGER = 19
US3_TRIGGER = 26

US1_ECHO = 16
US2_ECHO = 20
US3_ECHO = 21

TRIGGER_DURATION = 20e-6

HIGH = 1
LOW = 0

class Ultrasound:
  def __init__(self, triggerPin, echoPin):
    # Init GPIO
    self.io = pigpio.pi()
    if not self.io.connected:
      print("Unable to connection to pigpio daemon!")
      sys.exit(0)
    self.io.set_mode(triggerPin, pigpio.OUTPUT)
    self.io.set_mode(echoPin, pigpio.INPUT)
    
    self.io.set_pull_up_down(echoPin, pigpio.PUD_DOWN)

    self.triggerPin = triggerPin
    self.echoPin = echoPin

    # Ultrasound variables
    self.distance = 0.0
    self.rising_edge = 0

    # Set callback functions
    self.cbrise = self.io.callback(triggerPin, pigpio.RISING_EDGE, self.rising)
    self.cbfall = self.io.callback(echoPin, pigpio.FALLING_EDGE, self.falling)

    # Set timer to read ultrasound sensors every 100 ms.
    self.thread = threading.Thread(target=self.runcontinual)
    self.thread.start()

  # Pull trigger high for 10 us
  def trigger(self):
    self.io.write(self.triggerPin, HIGH)
    time.sleep(TRIGGER_DURATION)
    self.io.write(self.triggerPin, LOW)

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
    self.rising_edge = tick

  def falling(self, gpio, level, tick):
    t = float(tick - self.rising_edge) / 1e6
    distance = 343 * t/2 * 100 # in cm
    self.distance = distance

  def shutdown(self):
    self.stopcontinual()
    self.thread.join()

    self.cbrise.cancel()
    self.cbfall.cancel()
    self.io.stop()

if __name__ == "__main__":
  try:
    US1 = Ultrasound(US1_TRIGGER, US1_ECHO)
    US2 = Ultrasound(US2_TRIGGER, US2_ECHO)
    US3 = Ultrasound(US3_TRIGGER, US3_ECHO)
    while True:
      # print("Left: {:5.2f}, Mid: {:5.2f}, Right: {:5.2f}".format(US1.distance, US2.distance, US3.distance))
      print(US3.distance)
      time.sleep(0.1)
  except KeyboardInterrupt:
    print("Ending due to keyboard interrupt")
    US1.shutdown()
    US2.shutdown()
    US3.shutdown()
  except Exception as e:
    print("Ending due to exception: %s" % repr(e))
    US1.shutdown()
    US2.shutdown()
    US3.shutdown()