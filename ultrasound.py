from turtle import right
import pigpio
import sys
import time
import threading

US1_TRIGGER = 13
US2_TRIGGER = 19
US3_TRIGGER = 26
US1_ECHO = 16
US2_ECHO = 20
US3_ECHO = 21

US1_RISING_TIME = 0
US2_RISING_TIME = 0
US3_RISING_TIME = 0

GPIO_INDEX = {16: 0, 20: 1, 21: 2}

TRIGGER_DURATION = 0.00001

HIGH = 1
LOW = 0

class Ultrasound:
  def __init__(self):
    # Init GPIO
    self.io = pigpio.pi()
    if not self.io.connected:
      print("Unable to connection to pigpio daemon!")
      sys.exit(0)
    self.io.set_mode(US1_TRIGGER, pigpio.OUTPUT)
    self.io.set_mode(US2_TRIGGER, pigpio.OUTPUT)
    self.io.set_mode(US3_TRIGGER, pigpio.OUTPUT)
    self.io.set_mode(US1_ECHO, pigpio.INPUT)
    self.io.set_mode(US2_ECHO, pigpio.INPUT)
    self.io.set_mode(US3_ECHO, pigpio.INPUT)

    # Ultrasound variables
    self.distance = [0,0,0]
    self.rising_edge = [0,0,0]

    # Set callback functions
    self.cbrise = self.io.callback(US2_ECHO, pigpio.RISING_EDGE, self.rising)
    self.cbfall = self.io.callback(US2_ECHO, pigpio.FALLING_EDGE, self.falling)

    # Set timer to read ultrasound sensors every 100 ms.
    self.readUltrasound()

  # Pull trigger high for 10 us
  def trigger(self):
    self.io.write(US1_TRIGGER, HIGH)
    time.sleep(TRIGGER_DURATION)
    self.io.write(US1_TRIGGER, LOW)

    self.io.write(US2_TRIGGER, HIGH)
    time.sleep(TRIGGER_DURATION)
    self.io.write(US2_TRIGGER, LOW)

    self.io.write(US3_TRIGGER, HIGH)
    time.sleep(TRIGGER_DURATION)
    self.io.write(US3_TRIGGER, LOW)

  # read ultrasound sensors
  def readUltrasound(self):
    self.trigger()
    threading.Timer(1, self.readUltrasound).start()

  def shutdown(self):
    self.cbrise.cancel()
    self.cbfall.cancel()

  def rising(self, gpio, level, tick):
    print(gpio)
    if gpio == 16:
      self.rising_edge[0] = tick
    if gpio == 20:
      self.rising_edge[1] = tick
    if gpio == 21:
      self.rising_edge[2] = tick

  def falling(self, gpio, level, tick):
    index = GPIO_INDEX[gpio]
    t = (tick - self.rising_edge[index]) / 1e6
    distance = 343 * t/2 * 100 # in cm
    self.distance[index] = distance

def foo():
    print(time.ctime())
    threading.Timer(1, foo).start()

if __name__ == "__main__":
  US = Ultrasound()
  while True:
    print(US.distance)
    time.sleep(1)