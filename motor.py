#!/usr/bin/env python3

from turtle import right
import pigpio
import sys
import time

MTR1_LEGA = 8
MTR1_LEGB = 7

MTR2_LEGA = 5
MTR2_LEGB = 6

class Motor():
  def __init__(self):
    self.io = pigpio.pi()
    if not self.io.connected:
        print("Unable to connection to pigpio daemon!")
        sys.exit(0)

    # Set up the four pins as output (commanding the motors).
    self.io.set_mode(MTR1_LEGA, pigpio.OUTPUT)
    self.io.set_mode(MTR1_LEGB, pigpio.OUTPUT)
    self.io.set_mode(MTR2_LEGA, pigpio.OUTPUT)
    self.io.set_mode(MTR2_LEGB, pigpio.OUTPUT)

    # Prepare the PWM.  The range gives the maximum value for 100%
    # duty cycle, using integer commands (1 up to max).
    self.io.set_PWM_range(MTR1_LEGA, 255)
    self.io.set_PWM_range(MTR1_LEGB, 255)
    self.io.set_PWM_range(MTR2_LEGA, 255)
    self.io.set_PWM_range(MTR2_LEGB, 255)
    
    # Set the PWM frequency to 1000Hz.  You could try 500Hz or 2000Hz
    # to see whether there is a difference?
    self.io.set_PWM_frequency(MTR1_LEGA, 1000)
    self.io.set_PWM_frequency(MTR1_LEGB, 1000)
    self.io.set_PWM_frequency(MTR2_LEGA, 1000)
    self.io.set_PWM_frequency(MTR2_LEGB, 1000)

    # Clear all pins, just in case.
    self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
    self.io.set_PWM_dutycycle(MTR1_LEGB, 0)
    self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
    self.io.set_PWM_dutycycle(MTR2_LEGB, 0)

    print("GPIO ready...")

  def shutdown(self):
     # Clear all pins, just in case.
    self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
    self.io.set_PWM_dutycycle(MTR1_LEGB, 0)
    self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
    self.io.set_PWM_dutycycle(MTR2_LEGB, 0)

    self.io.stop()
  
  def set(self, leftdutycycle, rightdutycycle):
    if leftdutycycle > 1 or leftdutycycle < -1 or rightdutycycle > 1 or rightdutycycle < -1:
        print('Duty cycles ({}, {}) Out of Bounds'.format(leftdutycycle, rightdutycycle))
        raise ValueError

    leftpwm = leftdutycycle * 255
    rightpwm = rightdutycycle * 255
    
    if leftpwm >= 0:
      self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
      self.io.set_PWM_dutycycle(MTR1_LEGB, leftpwm)
    else:
      self.io.set_PWM_dutycycle(MTR1_LEGA, -leftpwm)
      self.io.set_PWM_dutycycle(MTR1_LEGB, 0)

    if rightpwm >= 0:
      self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
      self.io.set_PWM_dutycycle(MTR2_LEGB, rightpwm)
    else:
      self.io.set_PWM_dutycycle(MTR2_LEGA, -rightpwm)
      self.io.set_PWM_dutycycle(MTR2_LEGB, 0)

  def setlinear(self, speed, run=True, friction_compensation=0.361):
    # set run to True to actually set the pwm
    # if run is false, only return the pwm values
    # speed = 79.0 * (dutycycle - 0.361)
    # dutycycle = speed / 79.0 + 0.361

    # speed input range: [-50, 50]
    # speed unit: cm/s

    LINEAR_SLOPE = 79.0

    if speed > 0.0:
      dutycyle = speed/LINEAR_SLOPE + friction_compensation
    elif speed == 0.0:
      dutycyle = 0
    else:
      dutycyle = speed/LINEAR_SLOPE - friction_compensation
      
    if run:
      self.set(dutycyle, dutycyle)
    else:
      return dutycyle

  def setspin(self, speed, run=True, friction_compensation=0.435):
    # speed = 552 * (dutycycle - 0.412)
    # dutycycle = speed / 552 + 0.412
    # Angular velocity in degrees/s
    # Counterclockwise is positive
    # speed input range: [-324, 324]
    
    SPIN_SLOPE = 700

    if speed > 0.0:
      dutycycle = speed / SPIN_SLOPE + friction_compensation
    elif speed == 0.0:
      dutycycle = 0
    else:
      dutycycle = speed / SPIN_SLOPE - friction_compensation
      
    if run:
      self.set(-dutycycle, dutycycle)
    else:
      return dutycycle
    
  def setvel(self, linear, spin):
    # Linear portion of setvel
    dutycycle_linear = self.setlinear(linear, run=False, friction_compensation=0.361)
    # Spin portion of setvel
    # friction_compensation set to 20 because it's already moving
    dutycycle_spin = self.setspin(spin, run=False, friction_compensation=0.07)
    # Left and Right PWM
    leftdutycycle = dutycycle_linear - dutycycle_spin
    rightdutycycle = dutycycle_linear + dutycycle_spin
    
    self.set(leftdutycycle, rightdutycycle)
  
  def stop(self):
    self.set(0,0)
    
def line():
  motor.setlinear(25)
  time.sleep(1)
  motor.setspin(180)
  time.sleep(1)
  motor.setlinear(25)
  time.sleep(1)
  motor.setspin(180)
  time.sleep(1)

def triangle():  
  motor.setlinear(25)
  time.sleep(1)
  motor.setspin(120)
  time.sleep(1)
  motor.setlinear(25)
  time.sleep(1)
  motor.setspin(120)
  time.sleep(1)
  motor.setlinear(25)
  time.sleep(1)
  motor.setspin(120)
  time.sleep(1)

def circle ():
  motor.setvel(50*3.14/12, 360/12)
  time.sleep(12)  

if __name__ == "__main__":
  motor = Motor()
  try:
    motor.setspin(90)
    time.sleep(4)
  
    motor.shutdown()
  except KeyboardInterrupt:
    print("Ending due to keyboard interrupt")
    motor.shutdown()
  except Exception as e:
    print("Ending due to exception: %s" % repr(e))
    motor.shutdown()
