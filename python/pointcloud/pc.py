#!/usr/bin/env python3
# https://github.com/chobitsfan/oak_d_vins

import cv2
import depthai as dai
from collections import deque
import numpy as np
import rospy
# from sensor_msgs.msg import PointCloud, ChannelFloat32, Imu
# from geometry_msgs.msg import Point32, Vector3
import std_msgs.msg
import time
from msgs import *
from msgs import Vector3 as Point32


# def getMesh(camSocket, calibData):
#     M1 = np.array(calibData.getCameraIntrinsics(camSocket, 1280, 720))
#     d1 = np.array(calibData.getDistortionCoefficients(camSocket))
#     R1 = np.identity(3)
#     new_M1, validPixROI = cv2.getOptimalNewCameraMatrix(M1, d1, (1280,720), 0)
#     mapX, mapY = cv2.initUndistortRectifyMap(M1, d1, R1, new_M1, (1280, 720), cv2.CV_32FC1)

#     meshCellSize = 16
#     mesh0 = []
#     # Creates subsampled mesh which will be loaded on to device to undistort the image
#     for y in range(mapX.shape[0] + 1): # iterating over height of the image
#         if y % meshCellSize == 0:
#             rowLeft = []
#             for x in range(mapX.shape[1]): # iterating over width of the image
#                 if x % meshCellSize == 0:
#                     if y == mapX.shape[0] and x == mapX.shape[1]:
#                         rowLeft.append(mapX[y - 1, x - 1])
#                         rowLeft.append(mapY[y - 1, x - 1])
#                     elif y == mapX.shape[0]:
#                         rowLeft.append(mapX[y - 1, x])
#                         rowLeft.append(mapY[y - 1, x])
#                     elif x == mapX.shape[1]:
#                         rowLeft.append(mapX[y, x - 1])
#                         rowLeft.append(mapY[y, x - 1])
#                     else:
#                         rowLeft.append(mapX[y, x])
#                         rowLeft.append(mapY[y, x])
#             if (mapX.shape[1] % meshCellSize) % 2 != 0:
#                 rowLeft.append(0)
#                 rowLeft.append(0)

#             mesh0.append(rowLeft)

#     mesh0 = np.array(mesh0)
#     meshWidth = mesh0.shape[1] // 2
#     meshHeight = mesh0.shape[0]
#     mesh0.resize(meshWidth * meshHeight, 2)

#     mesh = list(map(tuple, mesh0))

#     return mesh, meshWidth, meshHeight, new_M1

base_ts = time.time() - dai.Clock.now().total_seconds()

# rospy.init_node('FeatureTracker', anonymous=True, disable_signals=True)
pp_pub = rospy.Publisher("/feature_tracker/feature", PointCloud, queue_size=10)
imu_pub = rospy.Publisher("/camera/imu", Imu, queue_size=50)

l_seq = -1
r_seq = -2
disp_seq = -3
l_prv_features = {}
r_prv_features = {}
prv_features_ts = 0
features_ts = 0

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
featureTrackerLeft = pipeline.create(dai.node.FeatureTracker)
featureTrackerRight = pipeline.create(dai.node.FeatureTracker)
imu = pipeline.create(dai.node.IMU)
depth = pipeline.create(dai.node.StereoDepth)

xoutTrackedFeaturesLeft = pipeline.create(dai.node.XLinkOut)
xoutTrackedFeaturesRight = pipeline.create(dai.node.XLinkOut)
xout_imu = pipeline.create(dai.node.XLinkOut)
xout_disp = pipeline.create(dai.node.XLinkOut)

xoutTrackedFeaturesLeft.setStreamName("trackedFeaturesLeft")
xoutTrackedFeaturesRight.setStreamName("trackedFeaturesRight")
xout_imu.setStreamName("imu")
xout_disp.setStreamName("disparity")

# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setFps(20)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setFps(20)
monoRight.setCamera("right")

featureTrackerLeft.initialConfig.setNumTargetFeatures(16*2)
featureTrackerRight.initialConfig.setNumTargetFeatures(16*2)

# enable ACCELEROMETER_RAW at 500 hz rate
imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 250)
# enable GYROSCOPE_RAW at 400 hz rate
imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 200)
# it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
# above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
imu.setBatchReportThreshold(1)
# maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
# if lower or equal to batchReportThreshold then the sending is always blocking on device
# useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
imu.setMaxBatchReports(10)

depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depth.setLeftRightCheck(True)
depth.setExtendedDisparity(False)
depth.setSubpixel(False)
depth.setDepthAlign(dai.RawStereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)
depth.setAlphaScaling(0)

# Linking
monoLeft.out.link(depth.left)
depth.rectifiedLeft.link(featureTrackerLeft.inputImage)
featureTrackerLeft.outputFeatures.link(xoutTrackedFeaturesLeft.input)

monoRight.out.link(depth.right)
depth.rectifiedRight.link(featureTrackerRight.inputImage)
featureTrackerRight.outputFeatures.link(xoutTrackedFeaturesRight.input)

imu.out.link(xout_imu.input)
depth.disparity.link(xout_disp.input)

# By default the least mount of resources are allocated
# increasing it improves performance
numShaves = 2
numMemorySlices = 2
featureTrackerLeft.setHardwareResources(numShaves, numMemorySlices)
featureTrackerRight.setHardwareResources(numShaves, numMemorySlices)

#featureTrackerConfig = featureTrackerRight.initialConfig.get()
#print("Press 's' to switch between Lucas-Kanade optical flow and hardware accelerated motion estimation!")

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    #calibData = device.readCalibration()
    #intri = calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, 640, 400)
    #fx = intri[0][0]
    #cx = intri[0][2]
    #fy = intri[1][1]
    #cy = intri[1][2]
    #print("left cam", fx ,fy, cx, cy)
    fx = 4.2007635451376063e+02
    cx = 3.0906091871893943e+02
    fy = 4.1970786993146550e+02
    cy = 2.0014588000163775e+02
    l_inv_k11 = 1.0 / fx
    l_inv_k13 = -cx / fx
    l_inv_k22 = 1.0 / fy
    l_inv_k23 = -cy / fy
    #intri = calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_C, 640, 400)
    #fx = intri[0][0]
    #cx = intri[0][2]
    #fy = intri[1][1]
    #cy = intri[1][2]
    #print("right cam", fx ,fy, cx, cy)
    fx = 4.1911737221609309e+02
    cx = 3.1094101439493375e+02
    fy = 4.1911302233054698e+02
    cy = 1.9966812289233278e+02
    r_inv_k11 = 1.0 / fx
    r_inv_k13 = -cx / fx
    r_inv_k22 = 1.0 / fy
    r_inv_k23 = -cy / fy

    # Output queues used to receive the results
    outputFeaturesLeftQueue = device.getOutputQueue("trackedFeaturesLeft", 8, False)
    outputFeaturesRightQueue = device.getOutputQueue("trackedFeaturesRight", 8, False)
    imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
    disp_queue = device.getOutputQueue(name="disparity", maxSize=8, blocking=False)

    device.getQueueEvents()

    while True:
        queue_names = device.getQueueEvents(("trackedFeaturesLeft", "trackedFeaturesRight", "imu", "disparity"))
        #print(queue_names)

        for queue_name in queue_names:
            if queue_name == "trackedFeaturesLeft":
                features_data = outputFeaturesLeftQueue.get()
                l_seq = features_data.getSequenceNum()
                l_features = features_data.trackedFeatures
                features_ts = features_data.getTimestamp().total_seconds()
            if queue_name == "trackedFeaturesRight":
                features_data = outputFeaturesRightQueue.get()
                r_seq = features_data.getSequenceNum()
                r_features = features_data.trackedFeatures
            if queue_name == "disparity":
                disp_data = disp_queue.get()
                disp_seq = disp_data.getSequenceNum()
                disp_frame = disp_data.getFrame()
            if queue_name == "imu":
                imuData = imuQueue.get()
                imuPackets = imuData.packets
                for imuPacket in imuPackets:
                    acc = imuPacket.acceleroMeter
                    gyro = imuPacket.gyroscope
                    imu_msg = Imu()
                    imu_msg.header = std_msgs.msg.Header()
                    imu_msg.header.stamp = rospy.Time.from_sec(base_ts + acc.getTimestamp().total_seconds())
                    imu_msg.header.frame_id = 'map'
                    imu_msg.linear_acceleration = Vector3(acc.z, acc.y, -acc.x)
                    imu_msg.angular_velocity = Vector3(gyro.z, gyro.y, -gyro.x)
                    imu_pub.publish(imu_msg)

            if l_seq == r_seq == disp_seq:
                start_ts = time.time()
                l_seq = -1
                r_seq = -2
                disp_seq = -3
                features = {}
                pp_msg = PointCloud()
                pp_msg.header = std_msgs.msg.Header()
                pp_msg.header.stamp = rospy.Time.from_sec(base_ts + features_ts)
                pp_msg.header.frame_id = 'map'
                pp_msg.channels = [ChannelFloat32(), ChannelFloat32(), ChannelFloat32(), ChannelFloat32(), ChannelFloat32(), ChannelFloat32()]
                for feature in l_features:
                    x = feature.position.x
                    y = feature.position.y
                    cur_un_pts = (l_inv_k11 * x + l_inv_k13, l_inv_k22 * y + l_inv_k23)
                    features[feature.id] = cur_un_pts
                    row = round(y)
                    col = round(x)
                    if col > 639:
                        col = 639
                    if row > 399:
                        row = 399
                    disp = disp_frame[row, col]
                    if disp > 0:
                        for r_feature in r_features:
                            dy = y - r_feature.position.y
                            dx = x - disp - r_feature.position.x
                            if dy * dy + dx * dx <= 16: #pair found
                                dt = features_ts - prv_features_ts
                                vx = 0
                                vy = 0
                                id = feature.id
                                if id in l_prv_features:
                                    vx = (cur_un_pts[0] - l_prv_features[id][0]) / dt
                                    vy = (cur_un_pts[1] - l_prv_features[id][1]) / dt
                                pp_msg.points.append(Point32(cur_un_pts[0], cur_un_pts[1], 1))
                                pp_msg.channels[0].values.append(id)
                                pp_msg.channels[1].values.append(0)
                                pp_msg.channels[2].values.append(x)
                                pp_msg.channels[3].values.append(y)
                                pp_msg.channels[4].values.append(vx)
                                pp_msg.channels[5].values.append(vy)

                                x = r_feature.position.x
                                y = r_feature.position.y
                                cur_un_pts = (r_inv_k11 * x + r_inv_k13, r_inv_k22 * y + r_inv_k23)
                                vx = 0
                                vy = 0
                                id = r_feature.id
                                if id in r_prv_features:
                                    vx = (cur_un_pts[0] - r_prv_features[id][0]) / dt
                                    vy = (cur_un_pts[1] - r_prv_features[id][1]) / dt
                                pp_msg.points.append(Point32(cur_un_pts[0], cur_un_pts[1], 1))
                                pp_msg.channels[0].values.append(feature.id)
                                pp_msg.channels[1].values.append(1)
                                pp_msg.channels[2].values.append(x)
                                pp_msg.channels[3].values.append(y)
                                pp_msg.channels[4].values.append(vx)
                                pp_msg.channels[5].values.append(vy)
                                break
                pp_pub.publish(pp_msg)
                print(pp_msg)
                # l_prv_features = features
                # prv_features_ts = features_ts
                # r_prv_features = {}
                # for feature in r_features:
                #     r_prv_features[feature.id] = (r_inv_k11 * feature.position.x + r_inv_k13, r_inv_k22 * feature.position.y + r_inv_k23)
                # print("total", len(pp_msg.points), "points, cost", time.time() - start_ts, "s")