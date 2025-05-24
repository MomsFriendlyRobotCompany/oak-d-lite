#!/usr/bin/env python3
# https://github.com/foxglove/foxglove-sdk/tree/main/schemas/proto/foxglove
#
# pip install opencv-contrib-python websockets
# pip install websockets numpy opencv-python protobuf
# pip install foxglove-websocket[examples]
#
import asyncio
import sys
import time
from base64 import b64encode
from foxglove_websocket import run_cancellable
from foxglove_websocket.server import FoxgloveServerListener
from foxglove_websocket.server import FoxgloveServer
from foxglove_schemas_protobuf.CameraCalibration_pb2 import CameraCalibration
from foxglove_schemas_protobuf.RawImage_pb2 import RawImage
from foxglove_schemas_protobuf.FrameTransform_pb2 import FrameTransform
# from foxglove_schemas_protobuf.SceneUpdate_pb2 import SceneUpdate
from google.protobuf.descriptor_pb2 import FileDescriptorSet
from google.protobuf.descriptor import FileDescriptor
# from pyquaternion import Quaternion
from squaternion import Quaternion
import numpy as np
import cv2
import depthai as dai
from threading import Thread
from msgs.AllMsgs_pb2 import Imu
from filters import CF
from collections import namedtuple
from colorama import Fore

frame = None
imu_msg = None
bgr2gray = lambda im: cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
deg2rad = np.pi / 180
gRun = True
time_now = time.time_ns()
# lidar = URG04LX()

vector_t = namedtuple("vector_t", "x y z")

def camera_worker(arg):
  global gRun
  global frame
  global imu_msg
  global time_now

  cf = CF(0.02)

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

    prev_ts = None
    dt = 0

    while gRun:
      imuData = imuQueue.get()  # blocking call, will wait until a new data has arrived

      imuPackets = imuData.packets
      for imuPacket in imuPackets:
        a = imuPacket.acceleroMeter
        # ts = a.getTimestamp()
        # now = ts.total_seconds()
        # print(now, time.time_ns())
        time_now = time.time_ns()
        if prev_ts is None:
          prev_ts = time_now
          continue
        else:
          # dt = (ts - prev_ts).total_seconds()
          dt = (time_now - prev_ts) / 1E9
          prev_ts = time_now

        # print(dt)
        g = imuPacket.gyroscope

        # rotate the IMU so x-forward and z-up
        a = vector_t(a.x,-a.y,-a.z)
        g = vector_t(g.x,-g.y,-g.z)
        # g = vector_t(g.x*deg2rad,-g.y*deg2rad,-g.z*deg2rad)

        # a = vector_t(-a.z,-a.x,a.y)
        # g = vector_t(-g.z,-g.x,g.y)
        # g = vector_t(-g.z*deg2rad,-g.x*deg2rad,g.y*deg2rad)

        imu_msg = Imu()
        imu_msg.header.frame_id = "camera"
        imu_msg.header.timestamp.FromNanoseconds(time_now)
        # imu_msg.frame_id = "camera"
        imu_msg.linear_acceleration.x = a.x
        imu_msg.linear_acceleration.y = a.y
        imu_msg.linear_acceleration.z = a.z
        imu_msg.angular_velocity.x = g.x
        imu_msg.angular_velocity.y = g.y
        imu_msg.angular_velocity.z = g.z

        # imu_msg.linear_acceleration.x = -a.z
        # imu_msg.linear_acceleration.y = -a.x
        # imu_msg.linear_acceleration.z = a.y
        # imu_msg.angular_velocity.x = g.x * np.pi / 180.0
        # imu_msg.angular_velocity.y = g.y * np.pi / 180.0
        # imu_msg.angular_velocity.z = g.z * np.pi / 180.0

        # q = Quaternion(axis=[0, 0, 1], angle=0.0)
        q = cf.update(a,g,dt) # FIXME: this sould be conjugate
        q = q.conjugate
        # print(q)
        imu_msg.orientation.x = q.x
        imu_msg.orientation.y = q.y
        imu_msg.orientation.z = q.z
        imu_msg.orientation.w = q.w

        # imuF = "{:9.6f}"
        # tsF  = "{:.3f}"

        # print(f"[ {acceleroTs} msec ]----------------------------")
        # print(f"Accel [m/s^2]: x: {imuF.format(a.x)} y: {imuF.format(a.y)} z: {imuF.format(a.z)}")
        # print(f"Gyro  [rad/s]: x: {imuF.format(g.x)} y: {imuF.format(g.y)} z: {imuF.format(g.z)}")

      # Instead of get (blocking), we use tryGet (non-blocking) which will return 
      # the available data or None otherwise
      # type: depthai.ImgFrame
      inLeft = qLeft.tryGet()
      inRight = qRight.tryGet()

      # https://docs.foxglove.dev/docs/visualization/message-schemas/raw-image
      # Frame of reference for the image. The origin of the frame is 
      # the optical center of the camera. +x points to the right in 
      # the image, +y points down, and +z points into the plane of 
      # the image.
      if inLeft and inRight:
        # print(inLeft.getCvFrame().shape)
        frame = np.hstack((inLeft.getCvFrame(), inRight.getCvFrame()))
        # frame = inLeft.getCvFrame()
        # print(frame.shape)
        # frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        # fps = monoLeft.getFps()
        # print(f"FPS: {fps}")


# Function to create a sample image
# def create_sample_image():
#   img = np.zeros((64, 64), dtype=np.uint8)
#   for i in range(64):
#     for j in range(64):
#       img[i, j] = ((i // 8) + (j // 8)) % 2 * 255
#   return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

def build_file_descriptor_set(message_class):
  """
  Build a FileDescriptorSet representing the message class and its dependencies.
  """
  file_descriptor_set = FileDescriptorSet()
  seen_dependencies: Set[str] = set()

  def append_file_descriptor(file_descriptor):
    for dep in file_descriptor.dependencies:
      if dep.name not in seen_dependencies:
        seen_dependencies.add(dep.name)
        append_file_descriptor(dep)
    file_descriptor.CopyToProto(file_descriptor_set.file.add())

  append_file_descriptor(message_class.DESCRIPTOR.file)
  return file_descriptor_set

def make_channel_info(topic, class_type):
  info = {
        "topic": topic,
        "encoding": "protobuf",
        "schemaName": class_type.DESCRIPTOR.full_name,
        "schema": b64encode(
            build_file_descriptor_set(class_type).SerializeToString()
        ).decode("ascii"),
        "schemaEncoding": "protobuf",
      }
  return info


async def main():
  class Listener(FoxgloveServerListener):
    async def on_subscribe(self, server, channel_id):
      print(f"{Fore.GREEN}First client subscribed to: {channel_id}{Fore.RESET}")

    async def on_unsubscribe(self, server, channel_id):
      print(f"{Fore.RED}Last client unsubscribed from: {channel_id}{Fore.RESET}")

  async with FoxgloveServer("0.0.0.0", 8765, "example server") as server:
    server.set_listener(Listener())
  #   chan_id = await server.add_channel( make_channel_info("lidar_msg", LaserScan) )
    image_id = await server.add_channel( make_channel_info("image_msg", RawImage) )
    cal_id = await server.add_channel( make_channel_info("cal_msg", CameraCalibration) )
    tf_id = await server.add_channel( make_channel_info("tf_msg", FrameTransform) )
    imu_id = await server.add_channel( make_channel_info("imu_msg", Imu) )

    i = 0
    while True:
      i += 1
      await asyncio.sleep(0.01)

      # scene_update = SceneUpdate()
      # entity = scene_update.entities.add()
      # entity.timestamp.FromNanoseconds(now)
      # entity.frame_id = "root"
      # cube = entity.cubes.add()
      # cube.size.x = 1
      # cube.size.y = 1
      # cube.size.z = 1
      # cube.pose.position.x = 2
      # cube.pose.position.y = 0
      # cube.pose.position.z = 0
      # q = Quaternion(axis=[0, 1, 1], angle=i * 0.1)
      # cube.pose.orientation.x = q.x
      # cube.pose.orientation.y = q.y
      # cube.pose.orientation.z = q.z
      # cube.pose.orientation.w = q.w
      # cube.color.r = 0.6
      # cube.color.g = 0.2
      # cube.color.b = 1
      # cube.color.a = 1

      # await server.send_message(chan_id, now, scene_update.SerializeToString())

      # Transform -----------------------------------------
      if imu_msg is None:
        continue
      # # qr = Quaternion(axis=[0, 0, 1], angle=0.0)
      # # # qr = Quaternion(axis=[0, 0, 1], angle=i*0.05)
      qr = Quaternion(
        x = imu_msg.orientation.x,
        y = imu_msg.orientation.y,
        z = imu_msg.orientation.z,
        w = imu_msg.orientation.w
      )
      # qfix = Quaternion(axis=[1,0,0], angle=np.pi/2)
      # qfix = Quaternion(axis=[0,1,0], angle=np.pi/2)
      qfix = Quaternion.from_angle_axis(np.pi, [0,0,1])
      q = qr * qfix
      # q = qr

      try:
        q = q.unit
      except ZeroDivisionError:
        continue

      T = FrameTransform()
      T.timestamp.FromNanoseconds(time_now)
      T.parent_frame_id = "root"
      T.child_frame_id = "camera"
      T.translation.x = 0
      T.translation.y = 0
      T.translation.z = 1
      T.rotation.x = q.x
      T.rotation.y = q.y
      T.rotation.z = q.z
      T.rotation.w = q.w
      # T.rotation.x = imu_msg.orientation.x
      # T.rotation.y = imu_msg.orientation.y
      # T.rotation.z = imu_msg.orientation.z
      # T.rotation.w = imu_msg.orientation.w
      # T.rotation.x = 0
      # T.rotation.y = 0
      # T.rotation.z = 0
      # T.rotation.w = 1
      await server.send_message(tf_id, time_now, T.SerializeToString())

      # Camera --------------------------------------------
      if frame is None:
        continue

      height, width = frame.shape
      # print(f"{height}x{width}")
      # print(f"{frame.shape}")
      img = RawImage()
      img.frame_id = "camera"
      img.width = width
      img.height = height
      # img.encoding = "rgb8"
      # img.step = width*3
      img.encoding = "mono8"
      img.step = width
      img.timestamp.FromNanoseconds(time_now)
      img.data = frame.tobytes()
      await server.send_message(image_id, time_now, img.SerializeToString())

      # IMU -----------------------------------------------
      # imu_msg.timestamp.FromNanoseconds(now)
      # imu_msg.header.timestamp.FromNanoseconds(now)
      await server.send_message(imu_id, time_now, imu_msg.SerializeToString())

      # Calibration ---------------------------------------
      # /camera/calibration
      # focal_length_mm = 1.3 # mm
      # sensor_width_mm = 4.69
      # fx = (focal_length_mm / sensor_width_mm) * width * 2
      # fy = (focal_length_mm / sensor_width_mm) * height

      HFOV = 73 * 2 # deg
      VFOV = 58 # deg
      fx = (width * 0.5) / np.tan(HFOV * 0.5 * np.pi/180)
      fy = (height * 0.5) / np.tan(VFOV * 0.5 * np.pi/180)

      cx = width / 2
      cy = height / 2
      Tx = 0.0
      Ty = 0.0
      cal = CameraCalibration(
          D=[0.0, 0.0, 0.0, 0.0, 0.0],
          K=[fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
          R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
          P=[fx, 0.0, cx, Tx, 0.0, fy, cy, Ty, 0.0, 0.0, 1.0, 0.0],
      )
      cal.frame_id="camera"
      cal.width=width
      cal.height=height
      cal.distortion_model="plumb_bob"
      cal.timestamp.FromNanoseconds(time_now)
      await server.send_message(cal_id, time_now, cal.SerializeToString())


if __name__ == "__main__":
  camera_thread = Thread(target=camera_worker, args=("hello",))
  camera_thread.start()

  run_cancellable(main())

  gRun = False
  camera_thread.join()