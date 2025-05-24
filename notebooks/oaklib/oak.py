import depthai as dai
import numpy as np
import cv2
from .helper import *
from tqdm import tqdm
from pathlib import Path
import yaml
from dataclasses import dataclass
from colorama import Fore
# from IPython.display import Video
# from squaternion import Quaternion
# from filters import *
# from datetime import timedelta
# import sys
# from collections import deque
# from matplotlib import pyplot as plt


def save_imu_params(filename, device=None):
    with dai.Device() as device:
        imuType = device.getConnectedIMU()
        imuFirmwareVersion = device.getIMUFirmwareVersion()
    # print(f"IMU type: {imuType}, firmware version: {imuFirmwareVersion}")

def save_camera_params(filename, device=None):
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
    json = None
    calibData = None

    if device is None:
        with dai.Device() as device:
            calibData = device.readCalibration()

    if calibData is None:
        return None
        
    json = calibData.eepromToJson()

    if json is not None:
        with open(filename, 'w') as fd:
            yaml.dump(json, fd)

    return json


def save_oak(path, imgs, imu):
    if not isinstance(path, Path):
        path = Path(path)

    ok = write_images(imgs, path / "images")
    if not ok:
        print(f"Error saving images to {path}")

    write_csv(path/"imu.csv", imu)
    save_camera_params(path / "oak.yml")

    print(f"Oak Camera Save -------------------")
    print(f" Location: {path}")
    print(f" Images: {len(imgs)}")
    print(f" IMU: {len(imu)}")

def load_oak(path):
    if not isinstance(path, Path):
        path = Path(path)

    imgs = read_images(path / "images")
    # if not ok:
    #     print(f"Error saving images to {path}")

    imu = read_csv(path / "imu.csv")
    # save_camera_params(path / "oak.yml")

    cam = OakCameraInfo(path / "oak.yml")

    print(f"Oak Camera Save -------------------")
    print(f" Location: {path}")
    print(f" Images: {len(imgs)}")
    print(f" IMU: {len(imu)}")

    return imgs, imu, cam

    
def grab_oak(numImgs=300, awb=True):
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
    
    # remove auto white balance at beginning
    if awb == False:
        controlIn = pipeline.create(dai.node.XLinkIn)
        controlIn.setStreamName('control')
        controlIn.out.link(monoLeft.inputControl)
    
    # IMU
    imu_speed = 201
    xoutimu = pipeline.create(dai.node.XLinkOut)
    xoutimu.setStreamName("imu")
    imu = pipeline.create(dai.node.IMU)
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, imu_speed)
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, imu_speed)
    # Not sure, setting these both the same
    # https://docs.luxonis.com/software/depthai/examples/imu_accelerometer_gyroscope/
    imu.setBatchReportThreshold(imu_speed // 30) # imu_speed / fps
    imu.setMaxBatchReports(20)
    imu.out.link(xoutimu.input)
    
    imu_data = []
    imgs = []
    
    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:
        
        # Disable AWB
        if awb == False:
            ctrl = dai.CameraControl()
            ctrl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.OFF)
            control_queue = device.getInputQueue('control')
            control_queue.send(ctrl)
    
        # Blockgin output queues will be used to get the grayscale frames from the outputs defined above
        # qLeft = device.getOutputQueue(name="left", maxSize=4, blocking=False)
        # qRight = device.getOutputQueue(name="right", maxSize=4, blocking=False)
        qLeft = device.getOutputQueue(name="left")
        qRight = device.getOutputQueue(name="right")
    
        # Output queue for imu bulk packets
        # imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
        imuQueue = device.getOutputQueue(name="imu", maxSize=50)
        for i in tqdm(range(numImgs)):
            # imuData = imuQueue.tryGet()
            imuData = imuQueue.get()
            # print(len(imuPackets))
            if imuData:
                imuPackets = imuData.packets
                for imuPacket in imuPackets:
                    a = imuPacket.acceleroMeter
                    g = imuPacket.gyroscope
                    ts = a.getTimestampDevice().total_seconds()
                    data = np.array([a.x,a.y,a.z,g.x,g.y,g.z,ts])
                    imu_data.append(data)
                
            # Instead of get (blocking), we use tryGet (non-blocking) which will return the available data or None otherwise
            # inLeft = qLeft.tryGet()
            # inRight = qRight.tryGet()
            inLeft = qLeft.get()
            inRight = qRight.get()
    
            if inLeft and inRight:
                frame = np.hstack((inLeft.getCvFrame(), inRight.getCvFrame()))
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                ts = inLeft.getTimestampDevice().total_seconds()
                imgs.append((frame, ts))

    return imgs, imu_data



# CameraParams = namedtuple("CameraParams","name K Rt hw")

class Distortion:
    def __init__(self, params):
        # k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4,τx,τy
        k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4,Tx,Ty = params
        self.radial = [k1,k2,k3,k4,k5,k6]
        self.tangental = [p1,p2]
        self.other = [s1,s2,s3,s4,Tx,Ty]
        self.cv2 = [k1,k2,p1,p2,k3,k4,k5,k6]

    def __str__(self):
        s = "Distortion\n"
        s += "  Radial: {0:.3f} {1:.3f} {2:.3f} {3:.3f} {4:.3f} {5:.3f}\n".format(*self.radial)
        s += "  Tangental: {0:.3f} {1:.3f}\n".format(*self.tangental)
        s += "  Other: {0:.3f} {1:.3f} {2:.3f} {3:.3f} {4:.3f} {5:.3f}\n".format(*self.other)
        return s

class Extrinsics:
    def __init__(self, R, t, relto):
        self.R = np.array(R)
        self.t = np.array([[t["x"]],[t["y"]],[t["z"]]])
        self.Rt = self.R @ self.t
        self.relto = relto

class OakCameraInfo:
    def __init__(self, filename):
        self.eprom = None

        try:
            with open(filename, "r") as fd:
                self.eprom = yaml.safe_load(fd)
        except yaml.YAMLError as err:
            print(err)
            return
        except Exception as err:
            print(err)
            return

        cam = self.eprom["stereoRectificationData"]
        self.rect1 = np.array(cam["rectifiedRotationLeft"])
        self.rect2 = np.array(cam["rectifiedRotationRight"])
        left = cam["leftCameraSocket"]
        right = cam["rightCameraSocket"]

        for i in range(3):
            id, cam = self.eprom["cameraData"][i]
            # print(cam)
            
                
            K = np.array(cam["intrinsicMatrix"])
            D = np.array(cam["distortionCoeff"])
            R = np.array(cam["extrinsics"]["rotationMatrix"])
            h = cam["height"]
            w = cam["width"]
            t = cam["extrinsics"]["translation"]
            fovH = cam["specHfovDeg"]
            dist = Distortion(cam["distortionCoeff"])
            

            if id == left:
                ext = Extrinsics(R,t,1)
                self.left = Camera(K,D,ext,h,w,fovH,dist)
                
            elif id == right:
                ext = Extrinsics(R,t,1)
                self.right = Camera(K,D,ext,h,w,fovH,dist)
            else:
                self.color = Camera(K,D,None,h,w,fovH,dist)

    
    def __str__(self):
        s = f"{Fore.BLUE}Left " + str(self.left) + f"\r{Fore.BLUE}Right " + str(self.right)
        # s = s.replace(' ','X')
        return s


@dataclass
class Camera:
    """
    Nice class for holding stereo camera info

    K: camera matrix
    d: distortion coefficients
    h: image height/rows
    w: image width/columns
    """
    K: np.ndarray # left camera matrix
    d: np.ndarray # distortion coefficients
    ext: int
    h: int # height/rows
    w: int # width/columns
    fovh: float
    dist: float

    def __str__(self):
        ms4 = lambda m: "    {}".format(str(m).replace('\n','\n    '))
        ms2 = lambda m: "  {}".format(str(m).replace('\n','\n  '))

        s = f'{Fore.BLUE}Camera[{self.h},{self.w}]----------------------{Fore.RESET}\n'
        s += f'  focalLength(x,y): {self.K[0,0]:.1f} {self.K[1,1]:.1f} px \n'
        s += f'  principlePoint(x,y): {self.K[0,2]:.1f} {self.K[1,2]:.1f} px\n'
        s += f'  distortionCoeffs: {self.d}\n'
        s += f"  FOVH: {self.fovh}\n"
        s += f"  Rotation: \n{ms4(self.ext.R)}\n"
        s += f"  Translation: \n{ms4(self.ext.t)}\n"
        # s += "\r"
        s += ms2(str(self.dist))
        # s += str(self.dist)
        return s