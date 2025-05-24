#!/usr/bin/env python3
# import cv2
import depthai as dai
import time
import math

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
imu = pipeline.create(dai.node.IMU)
xlinkOut = pipeline.create(dai.node.XLinkOut)

xlinkOut.setStreamName("imu")

# enable ACCELEROMETER_RAW at 500 hz rate
imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 500)
# enable GYROSCOPE_RAW at 400 hz rate
imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
# it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
# above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
imu.setBatchReportThreshold(1)
# maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
# if lower or equal to batchReportThreshold then the sending is always blocking on device
# useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
imu.setMaxBatchReports(10)

# Link plugins IMU -> XLINK
imu.out.link(xlinkOut.input)

# Pipeline is defined, now we can connect to the device
with dai.Device(pipeline) as device:

    def timeDeltaToMilliS(delta) -> float:
        return delta.total_seconds()*1000

    # Output queue for imu bulk packets
    imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
    baseTs = None

    try:
        while True:
            imuData = imuQueue.get()  # blocking call, will wait until a new data has arrived

            imuPackets = imuData.packets
            for imuPacket in imuPackets:
                a = imuPacket.acceleroMeter
                g = imuPacket.gyroscope

                acceleroTs = a.getTimestampDevice()
                if baseTs is None:
                    baseTs = acceleroTs
                acceleroTs = timeDeltaToMilliS(acceleroTs - baseTs)

                imuF = "{:9.6f}"
                tsF  = "{:.3f}"

                print(f"[ {tsF.format(acceleroTs)} msec ]----------------------------")
                print(f"Accel [m/s^2]: x: {imuF.format(a.x)} y: {imuF.format(a.y)} z: {imuF.format(a.z)}")
                print(f"Gyro  [rad/s]: x: {imuF.format(g.x)} y: {imuF.format(g.y)} z: {imuF.format(g.z)}")
    except KeyboardInterrupt:
        print("\rbye ...")


