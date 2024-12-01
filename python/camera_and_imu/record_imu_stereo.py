#!/usr/bin/env python3
import depthai as dai
import numpy as np
import cv2
from datetime import timedelta
import sys

imuinfo = {}

# device = dai.Device()

# imuinfo["type"] = device.getConnectedIMU()
# imuinfo["firmware"] = device.getIMUFirmwareVersion()
# print(f"IMU type: {imuType}, firmware version: {imuFirmwareVersion}")


# Create pipeline
pipeline = dai.Pipeline()

# Stereo Cameras
xoutLeft = pipeline.create(dai.node.XLinkOut)
xoutLeft.setStreamName('left')
monoLeft = pipeline.create(dai.node.MonoCamera)
monoLeft.setCamera("left")
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
monoLeft.out.link(xoutLeft.input)

xoutRight = pipeline.create(dai.node.XLinkOut)
xoutRight.setStreamName('right')
monoRight = pipeline.create(dai.node.MonoCamera)
monoRight.setCamera("right")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
monoRight.out.link(xoutRight.input)

# IMU
xoutimu = pipeline.create(dai.node.XLinkOut)
xoutimu.setStreamName("imu")
imu = pipeline.create(dai.node.IMU)
imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 500)
imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
imu.setBatchReportThreshold(1)
imu.setMaxBatchReports(10)
imu.out.link(xoutimu.input)


# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    # Output queues will be used to get the grayscale frames from the outputs defined above
    qLeft = device.getOutputQueue(name="left", maxSize=4, blocking=False)
    qRight = device.getOutputQueue(name="right", maxSize=4, blocking=False)

    # Output queue for imu bulk packets
    imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)

    while True:
        imuData = imuQueue.get()  # blocking call, will wait until a new data has arrived

        imuPackets = imuData.packets
        for imuPacket in imuPackets:
            a = imuPacket.acceleroMeter
            g = imuPacket.gyroscope

            acceleroTs = a.getTimestampDevice()

            imuF = "{:9.6f}"
            tsF  = "{:.3f}"

            print(f"[ {acceleroTs} msec ]----------------------------")
            print(f"Accel [m/s^2]: x: {imuF.format(a.x)} y: {imuF.format(a.y)} z: {imuF.format(a.z)}")
            print(f"Gyro  [rad/s]: x: {imuF.format(g.x)} y: {imuF.format(g.y)} z: {imuF.format(g.z)}")

        # Instead of get (blocking), we use tryGet (non-blocking) which will return the available data or None otherwise
        inLeft = qLeft.tryGet()
        inRight = qRight.tryGet()

        if inLeft and inRight:
            frame = np.hstack((inLeft.getCvFrame(), inRight.getCvFrame()))
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            fps = monoLeft.getFps()
            cv2.putText(frame,
                f"Fps: {fps:.2f}",
                (2, frame.shape[0] - 4),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.4, (255,0,0))
            cv2.imshow("camera", frame)

        if cv2.waitKey(1) == ord('q'):
            break