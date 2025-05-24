#!/usr/bin/env python3
# https://docs.luxonis.com/software/depthai/examples/calibration_reader/

import depthai as dai
import numpy as np
from thebrian.oak import camera_info
from pprint import pprint
np.set_printoptions(suppress=True)
np.set_printoptions(precision=1)

device = dai.Device()
info = camera_info(device)
pprint(info)

# calibData = device.readCalibration()
# print(calibData.getCameraExtrinsics(dai.CameraBoardSocket.CAM_B))
# print(calibData.getCameraToImuExtrinsics(dai.CameraBoardSocket.CAM_B))
# print(calibData.getEepromData(dai.CameraBoardSocket.CAM_B))
# print(calibData.getFov(dai.CameraBoardSocket.CAM_B))

# def getDefaultIntrinsics(calibData, cam):
#     M, width, height = calibData.getDefaultIntrinsics(cam)
#     print(f"{cam} @ {width} x {height} FOV {calibData.getFov(cam):.1f} deg\nIntrinsics:")
#     print(np.array(M))
#     print()
#     return np.array(M)

# def getIntrinsics(calibData, cam, width, height):
#     M = calibData.getCameraIntrinsics(cam, width, height)
#     print(f"{cam} @ {width} x {height}")
#     print(np.array(M))
#     print()
#     return np.array(M)

# def getDistortion(calibData, cam):
#     D = np.array(calibData.getDistortionCoefficients(cam))
#     print("Distortion Coefficients:")
#     [print(f"{name}: {value}") for (name, value) in zip(["k1","k2","p1","p2","k3","k4","k5","k6","s1","s2","s3","s4","τx","τy"],[str(val) for val in D])]
#     print()
#     return D

# device = dai.Device()
# calibData = device.readCalibration()
# print(f"{calibData.getEepromData().boardName}")
# # print(calibData)
# print(dir(calibData))

# print("-----------------")
# pprint(calibData.eepromToJson())

# print("-----------------")
# getDefaultIntrinsics(calibData, dai.CameraBoardSocket.CAM_A)
# # getIntrinsics(calibData, dai.CameraBoardSocket.CAM_A, 3840, 2160)
# # getIntrinsics(calibData, dai.CameraBoardSocket.CAM_A, 4056, 3040)
# getDistortion(calibData, dai.CameraBoardSocket.CAM_A)

# print("-----------------")
# getDefaultIntrinsics(calibData, dai.CameraBoardSocket.CAM_B)
# # getIntrinsics(calibData, dai.CameraBoardSocket.CAM_B, 640, 400)
# getDistortion(calibData, dai.CameraBoardSocket.CAM_B)

# print("-----------------")
# getDefaultIntrinsics(calibData, dai.CameraBoardSocket.CAM_C)
# # getIntrinsics(calibData, dai.CameraBoardSocket.CAM_C, 640, 400)
# getDistortion(calibData, dai.CameraBoardSocket.CAM_C)

# # M_right = np.array(calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_C, 1280, 720))
# # print("RIGHT Camera resized intrinsics... 1280 x 720")
# # print(M_right)

# # D_left = np.array(calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_B))
# # print("LEFT Distortion Coefficients...")
# # [print(name+": "+value) for (name, value) in zip(["k1","k2","p1","p2","k3","k4","k5","k6","s1","s2","s3","s4","τx","τy"],[str(data) for data in D_left])]

# # D_right = np.array(calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_C))
# # print("RIGHT Distortion Coefficients...")
# # [print(name+": "+value) for (name, value) in zip(["k1","k2","p1","p2","k3","k4","k5","k6","s1","s2","s3","s4","τx","τy"],[str(data) for data in D_right])]

# # print(f"RGB FOV {calibData.getFov(dai.CameraBoardSocket.CAM_A)}, Mono FOV {calibData.getFov(dai.CameraBoardSocket.CAM_B)}")

# R1 = np.array(calibData.getStereoLeftRectificationRotation())
# R2 = np.array(calibData.getStereoRightRectificationRotation())
# M_right = np.array(calibData.getCameraIntrinsics(calibData.getStereoRightCameraId(), 1280, 720))

# H_left = np.matmul(np.matmul(M_right, R1), np.linalg.inv(M_left))
# print("LEFT Camera stereo rectification matrix...")
# print(H_left)

# H_right = np.matmul(np.matmul(M_right, R1), np.linalg.inv(M_right))
# print("RIGHT Camera stereo rectification matrix...")
# print(H_right)

# lr_extrinsics = np.array(calibData.getCameraExtrinsics(dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C))
# print("Transformation matrix of where left Camera is W.R.T right Camera's optical center")
# print(lr_extrinsics)

# l_rgb_extrinsics = np.array(calibData.getCameraExtrinsics(dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_A))
# print("Transformation matrix of where left Camera is W.R.T RGB Camera's optical center")
# print(l_rgb_extrinsics)

# # print(calibData.getCameraExtrinsics(dai.CameraBoardSocket.CAM_B))
# # print(calibData.getCameraToImuExtrinsics(dai.CameraBoardSocket.CAM_B))
# # print(calibData.getEepromData(dai.CameraBoardSocket.CAM_B))
# # print(calibData.getFov(dai.CameraBoardSocket.CAM_B))

# # D_left = np.array(calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_B))
# # print("LEFT Distortion Coefficients...")
# # [print(name+": "+value) for (name, value) in zip(["k1","k2","p1","p2","k3","k4","k5","k6","s1","s2","s3","s4","τx","τy"],[str(data) for data in D_left])]

# # D_right = np.array(calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_C))
# # print("RIGHT Distortion Coefficients...")
# # [print(name+": "+value) for (name, value) in zip(["k1","k2","p1","p2","k3","k4","k5","k6","s1","s2","s3","s4","τx","τy"],[str(data) for data in D_right])]

# # print("-----------------")
# # intri = calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, 640, 400)
# # # fx = intri[0][0]
# # # cx = intri[0][2]
# # # fy = intri[1][1]
# # # cy = intri[1][2]
# # # print("left cam", fx ,fy, cx, cy)
# # print(np.array(intri))

# # intri = calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_C, 640, 400)
# # print(np.array(intri))
# # # fx = intri[0][0]
# # # cx = intri[0][2]
# # # fy = intri[1][1]
# # # cy = intri[1][2]
# # # print("right cam", fx ,fy, cx, cy)

# # # print(intri)
# # # print(dir(intri))