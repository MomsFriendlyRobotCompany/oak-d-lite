#!/usr/bin/env python3
# https://github.com/foxglove/foxglove-sdk/tree/main/schemas/proto/foxglove
#
# pip install opencv-contrib-python websockets
# pip install websockets numpy opencv-python protobuf==5.29.3
# pip install foxglove-websocket[examples]
#
import asyncio
import sys
import time
from base64 import b64encode
from foxglove_websocket import run_cancellable
from foxglove_websocket.server import FoxgloveServerListener
from foxglove_websocket.server import FoxgloveServer
# from foxglove_schemas_protobuf.FrameTransform_pb2 import FrameTransform
from foxglove_schemas_protobuf.SceneUpdate_pb2 import SceneUpdate
from google.protobuf.descriptor_pb2 import FileDescriptorSet
from google.protobuf.descriptor import FileDescriptor
from squaternion import Quaternion as sQuaternion
import numpy as np
import depthai as dai
from threading import Thread
from colorama import Fore
from gecko_messages import *

class CF:
  """
  Performance increases if the bias is removed and any scaling performed
  on the sensor readings before using the filter.
  
  For accelerometers, there is no need to remove gravity since the 
  values will be normalized.
  
  https://ahrs.readthedocs.io/en/latest/filters/complementary.html
  """
  def __init__(self,aa=0.02):
    self.q = sQuaternion()
    self.aa = aa
      
  def update(self, a, g, dt, m=None):
    """
    Update attitude quaternion with accelerometer and gyroscope 
    readings. Optionally, a magnitometer can be used to better
    recover the yaw orientation.
    
    a: accel [doesn't matter], gets normalized
    g: gyro [rad/sec]
    m: mag [doesn't matter], gets normalized
    dt: time step [sec]
    """
    dt = float(dt)
    qw = self.q + 0.5*dt*self.q*sQuaternion(0,g.x, g.y, g.z)

    a = np.array([a.x, a.y, a.z])
    a = a / np.linalg.norm(a)
    
    # https://academicworks.cuny.edu/cgi/
    #    viewcontent.cgi?referer=&httpsredir=1&article=1247&context=cc_pubs
    # Fig 2, pg 19315
    # rot = self.q.to_rot() # value?
    # a = rot @ a           # value?

    ax,ay,az = a
    roll = np.arctan2(ay,az)
    pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))
    
    if m is None:
      # qq = qw.conjugate
      # r,p,y = qw.to_euler(degrees=False)
      # yaw = y
      yaw = 0.0
    else:
      mx,my,mz = m / np.linalg.norm(m)
      x = mx*np.cos(pitch)+mz*np.sin(pitch)
      y = mx*np.sin(roll)*np.sin(pitch)+my*np.cos(roll)-mz*np.sin(roll)*np.cos(pitch)
      yaw = np.pi/2 - np.arctan2(y, x) # or np.atan2(x,y) is the same thing
        
    qam = sQuaternion.from_euler(roll, pitch, yaw)
    
    # print("self.q: ",type(self.q))
    self.q = (1-self.aa) * qw + self.aa * qam
    # print("self.q: ",type(self.q))
    return self.q



imu_msg = None
gRun = True
time_now = time.time_ns()

cf = CF(0.02)

def camera_worker(arg):
  global gRun
  global frame
  global imu_msg
  global time_now
  global cf

  # Create pipeline
  pipeline = dai.Pipeline()

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
    # Output queue for imu bulk packets
    imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)

    prev_ts = None
    dt = 0

    while gRun:
      imuData = imuQueue.get()  # blocking call, will wait until a new data has arrived

      imuPackets = imuData.packets
      for imuPacket in imuPackets:
        time_now = time.time_ns()
        if prev_ts is None:
          prev_ts = time_now
          continue
        else:
          dt = (time_now - prev_ts) / 1E9
          prev_ts = time_now

        a = imuPacket.acceleroMeter
        g = imuPacket.gyroscope

        # rotate the IMU so x-forward and z-up
        a = vector_t(a.x,-a.y,-a.z)
        g = vector_t(g.x,-g.y,-g.z)

        cf.update(a,g,dt) # FIXME: this sould be conjugate

        imu_msg = Imu()
        imu_msg.header.frame_id = "imu"
        imu_msg.header.timestamp.FromNanoseconds(time_now)
        imu_msg.linear_acceleration.x = a.x
        imu_msg.linear_acceleration.y = a.y
        imu_msg.linear_acceleration.z = a.z
        imu_msg.angular_velocity.x = g.x
        imu_msg.angular_velocity.y = g.y
        imu_msg.angular_velocity.z = g.z

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
    scene_id = await server.add_channel( make_channel_info("scene", SceneUpdate) )
    # tf_id = await server.add_channel( make_channel_info("tf_msg", FrameTransform) )
    imu_id = await server.add_channel( make_channel_info("imu_msg", Imu) )

    i = 0
    while True:
      i += 1
      await asyncio.sleep(0.01)
      now = time.time_ns()

      scene_update = SceneUpdate()
      entity = scene_update.entities.add()
      entity.timestamp.FromNanoseconds(now)
      entity.frame_id = "root"
      cube = entity.cubes.add()
      cube.size.x = .1
      cube.size.y = .1
      cube.size.z = .1
      cube.pose.position.x = 0
      cube.pose.position.y = 0
      cube.pose.position.z = 0

      q = cf.q.conjugate
      cube.pose.orientation.x = q.x
      cube.pose.orientation.y = q.y
      cube.pose.orientation.z = q.z
      cube.pose.orientation.w = q.w
      cube.color.r = 0.6
      cube.color.g = 0.2
      cube.color.b = 1
      cube.color.a = 1

      await server.send_message(scene_id, now, scene_update.SerializeToString())

      # Transform -----------------------------------------
      if imu_msg is None:
        continue
      # # qr = Quaternion(axis=[0, 0, 1], angle=0.0)
      # # # qr = Quaternion(axis=[0, 0, 1], angle=i*0.05)
      # qr = Quaternion(
      #   x = imu_msg.orientation.x,
      #   y = imu_msg.orientation.y,
      #   z = imu_msg.orientation.z,
      #   w = imu_msg.orientation.w
      # )
      # qfix = Quaternion(axis=[1,0,0], angle=np.pi/2)
      # qfix = Quaternion(axis=[0,1,0], angle=np.pi/2)
      # qfix = Quaternion.from_angle_axis(np.pi, [0,0,1])
      # q = qr * qfix
      # q = qr

      # qr = quaternion_t(
      #   imu_msg.orientation.w,
      #   imu_msg.orientation.x,
      #   imu_msg.orientation.y,
      #   imu_msg.orientation.z,
      # )

      # try:
      #   q = q.unit
      # except ZeroDivisionError:
      #   continue

      # T = FrameTransform()
      # T.timestamp.FromNanoseconds(time_now)
      # T.parent_frame_id = "root"
      # T.child_frame_id = "camera"
      # T.translation.x = 0
      # T.translation.y = 0
      # T.translation.z = 1
      # # T.rotation.x = q.x
      # # T.rotation.y = q.y
      # # T.rotation.z = q.z
      # # T.rotation.w = q.w
      # # T.rotation.x = imu_msg.orientation.x
      # # T.rotation.y = imu_msg.orientation.y
      # # T.rotation.z = imu_msg.orientation.z
      # # T.rotation.w = imu_msg.orientation.w
      # T.rotation.x = 0
      # T.rotation.y = 0
      # T.rotation.z = 0
      # T.rotation.w = 1
      # await server.send_message(tf_id, time_now, T.SerializeToString())

      # IMU -----------------------------------------------
      # imu_msg.timestamp.FromNanoseconds(now)
      # imu_msg.header.timestamp.FromNanoseconds(now)
      await server.send_message(imu_id, time_now, imu_msg.SerializeToString())



if __name__ == "__main__":
  camera_thread = Thread(target=camera_worker, args=("hello",))
  camera_thread.start()

  run_cancellable(main())

  gRun = False
  camera_thread.join()