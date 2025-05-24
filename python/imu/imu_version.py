#!/usr/bin/env python3
import depthai as dai
from thebrian.oak import imu_info
from pprint import pprint
"""
https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi270/

BMI270:
Note that BMI279 "rounds down" the input frequency to the next 
available frequency. For example, if you set the frequency to 
99 it will round it to 50Hz. Additionally, the current max 
frequency of ~250 Hz is set when the input is >400Hz.

- Accelerometer: 25Hz, 50Hz, 100Hz, 200Hz, 250Hz
- Gyroscope: 25Hz, 50Hz, 100Hz, 200Hz, 250Hz
"""


device = dai.Device()
info = imu_info(device)
pprint(info)

# imuType = device.getConnectedIMU()
# imuFirmwareVersion = device.getIMUFirmwareVersion()
# print(f"IMU type: {imuType}, firmware version: {imuFirmwareVersion}")
