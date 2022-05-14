import time
import math
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

mpu = MPU9250(
    address_ak=AK8963_ADDRESS, 
    address_mpu_master=MPU9050_ADDRESS_68, # In 0x68 Address
    address_mpu_slave=None, 
    bus=1,
    gfs=GFS_1000, 
    afs=AFS_8G, 
    mfs=AK8963_BIT_16, 
    mode=AK8963_MODE_C100HZ)

# mpu.calibrate() # Calibrate sensors
mpu.calibrateAK8963() # Calibrate sensors
mpu.configure() # The calibration function resets the sensors, so you need to reconfigure them

magScale = mpu.magScale # Get magnetometer soft iron distortion
mbias = mpu.mbias # Get magnetometer hard iron distortion
print(magScale, mbias)
mpu.configure() # Apply the settings to the registers.

while True:
  mx, my, mz = mpu.readMagnetometerMaster()
  print(math.atan2(my,mx) * 180 / math.pi)

  # print("|.....MPU9250 in 0x68 Address.....|")
  # print("Accelerometer", mpu.readAccelerometerMaster())
  # print("Gyroscope", mpu.readGyroscopeMaster())
  # print("Magnetometer", mpu.readMagnetometerMaster())
  # print("Temperature", mpu.readTemperatureMaster())
  print("\n")

  time.sleep(0.1)