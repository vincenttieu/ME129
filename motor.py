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
    if leftdutycycle >= 0:
      self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
      self.io.set_PWM_dutycycle(MTR1_LEGB, leftdutycycle)
      
    else:
      self.io.set_PWM_dutycycle(MTR1_LEGA, 255)
      self.io.set_PWM_dutycycle(MTR1_LEGB, 255+leftdutycycle)

    if rightdutycycle >= 0:
      self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
      self.io.set_PWM_dutycycle(MTR2_LEGB, rightdutycycle)
    else:
      self.io.set_PWM_dutycycle(MTR2_LEGA, 255)
      self.io.set_PWM_dutycycle(MTR2_LEGB, 255+rightdutycycle)

  def setlinear(self, speed, run=True, friction_compensation=92):
    # set run to True to actually set the pwm
    # if run is false, only return the pwm values
    # speed = 0.31 * (pwm - 92)
    if speed > 0.0:
      pwm = speed/0.31 + friction_compensation
    elif speed == 0.0:
      pwm = 0
    else:
      pwm = speed/0.31 - friction_compensation
      
    if run:
      # Safety to ensure pwm values don't exceed 255
      if pwm > 255 or pwm < -255:
        print('Calculated PWM ({}) Out of Bounds'.format(pwm))
        raise KeyboardInterrupt
      self.set(pwm, pwm)
    else:
      return pwm

  def setspin(self, speed, run=True, friction_compensation=105):
    # speed = 1.26 * (pwm - 105)
    # Angular velocity in degrees/s
    # Counterclockwise is positive
    
    if speed > 0.0:
      pwm = speed/1.26 + friction_compensation
    elif speed == 0.0:
      pwm = 0
    else:
      pwm = speed/1.26 - friction_compensation
      
    if run:
      if pwm > 255 or pwm < -255:
        print('Calculated PWM ({}) Out of Bounds'.format(pwm))
        raise KeyboardInterrupt
      self.set(-pwm, pwm)
    else:
      return pwm
    
  def setvel(self, linear, spin):
    # Linear portion of setvel
    pwm_linear = self.setlinear(linear, run=False, friction_compensation=100)
    print(pwm_linear)
    # Spin portion of setvel
    pwm_spin = self.setspin(spin, run=False, friction_compensation=20)
    print(pwm_spin)
    # Left and Right PWM
    leftpwm = pwm_linear - pwm_spin
    rightpwm = pwm_linear + pwm_spin
    
    if (leftpwm > 255 or leftpwm < -255) or (rightpwm > 255 or rightpwm < -255):
      print('Calculated PWM ({}, {}) Out of Bounds'.format(leftpwm, rightpwm))
      raise KeyboardInterrupt
    
    self.set(leftpwm, rightpwm)
    print(leftpwm)
    print(rightpwm)
    
    # Circle test
    #leftspeed = 198/6
    #rightspeed = 116/6
    #leftpwm = leftspeed/0.31 + 92
    #rightpwm = rightspeed/0.31 + 92
    #self.set(leftpwm, rightpwm)

def p5():
  slow = 150
  less_slow = 200
  medium = 250

  motor.setlinear(50)
  time.sleep(1)

def p6():
  slow = 150
  less_slow = 200
  medium = 250
  motor.setspin(medium)
  time.sleep(10)

def p7():
  # motor.setlinear(150)
  # time.sleep(1)
  # motor.setspin(150)
  # time.sleep(1.3)
  # motor.setlinear(150)
  # time.sleep(1)

  motor.setlinear(150)
  time.sleep(1)
  motor.setspin(150)
  time.sleep(0.9)
  motor.setlinear(150)
  time.sleep(1)
  motor.setspin(150)
  time.sleep(0.9)
  motor.setlinear(150)
  time.sleep(1)
  motor.setspin(150)
  time.sleep(0.9)


if __name__ == "__main__":
  motor = Motor()
  try:
    motor.setvel(157/12,360/12)
    time.sleep(12)
    motor.shutdown()  
  except KeyboardInterrupt:
    motor.shutdown()

  except Exception as e:
    print("Ending due to exception: %s" % repr(e))
    motor.shutdown()
