#!/usr/bin/env python3
import depthai as dai
import numpy as np
import cv2
from datetime import timedelta

# imuinfo = {}

device = dai.Device()

# imuinfo["type"] = device.getConnectedIMU()
# imuinfo["firmware"] = device.getIMUFirmwareVersion()
# print(f"IMU type: {imuType}, firmware version: {imuFirmwareVersion}")

# Really?
# if imuType != "BNO086":
#     print("Rotation vector output is supported only by BNO086!")
#     exit(0)

pipeline = dai.Pipeline()

sync = pipeline.create(dai.node.Sync)

color = pipeline.create(dai.node.ColorCamera)
color.setCamera("color")

# monoLeft = pipeline.create(dai.node.MonoCamera)
# monoLeft.setCamera("left")
# monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
# xoutLeft = pipeline.create(dai.node.XLinkOut)

# monoRight = pipeline.create(dai.node.MonoCamera)
# monoRight.setCamera("right")
# monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
# xoutRight = pipeline.create(dai.node.XLinkOut)

imu = pipeline.create(dai.node.IMU)
imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 500)
imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
imu.setBatchReportThreshold(1)
imu.setMaxBatchReports(10)
xoutImu = pipeline.create(dai.node.XLinkOut)
xoutImu.setStreamName("imu")

xoutGrp = pipeline.create(dai.node.XLinkOut)
xoutGrp.setStreamName("xout")

sync.setSyncThreshold(timedelta(milliseconds=10))
sync.setSyncAttempts(-1)

color.video.link(sync.inputs["video"])
# monoLeft.video.link(sync.inputs["video"])
# monoRight.video.link(sync.inputs["video"])
imu.out.link(sync.inputs["imu"])

sync.out.link(xoutGrp.input)


with device:
    device.startPipeline(pipeline)
    groupQueue = device.getOutputQueue("xout", 3, True)

    try:
        while True:
            groupMessage = groupQueue.get()
            imuMessage = groupMessage["imu"]
            colorMessage = groupMessage["video"]

            # print(dir(imuMessage))
            imuPackets = imuMessage.packets
            for imuPacket in imuPackets:
                a = imuPacket.acceleroMeter
                g = imuPacket.gyroscope
                acceleroTs = a.getTimestampDevice().total_seconds()

                imuF = "{:9.6f}"
                tsF  = "{:.3f}"
                # print()
                print("Device timestamp imu: " + str(imuMessage.getTimestampDevice()))
                print("Device timestamp video:" + str(colorMessage.getTimestampDevice()))
                # latestRotationVector = imuMessage.packets[-1].rotationVector
                # imuF = "{:.4f}"

                print(f"[ {tsF.format(acceleroTs)} ]----------------------------")
                print(f"Accel [m/s^2]: x: {imuF.format(a.x)} y: {imuF.format(a.y)} z: {imuF.format(a.z)}")
                print(f"Gyro  [rad/s]: x: {imuF.format(g.x)} y: {imuF.format(g.y)} z: {imuF.format(g.z)}")

            # cv2.imshow("video", colorMessage.getCvFrame())
            # if cv2.waitKey(1) == ord('q'):
            #     break

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(f"Error: {e}\n")

    finally:
        print("\rbye ...")
