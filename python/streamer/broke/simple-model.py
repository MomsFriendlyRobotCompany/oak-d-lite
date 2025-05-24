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
# from foxglove_schemas_protobuf.CameraCalibration_pb2 import CameraCalibration
# from foxglove_schemas_protobuf.RawImage_pb2 import RawImage
# from foxglove_schemas_protobuf.FrameTransform_pb2 import FrameTransform
from foxglove_schemas_protobuf.SceneUpdate_pb2 import SceneUpdate
from google.protobuf.descriptor_pb2 import FileDescriptorSet
from google.protobuf.descriptor import FileDescriptor
from pyquaternion import Quaternion
# from squaternion import Quaternion
import numpy as np
from colorama import Fore
time_now = time.time_ns()

xml = """
<!-- simple_arm.urdf -->
<robot name="simple_arm">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>

  <!-- First link -->
  <link name="link1">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <!-- First joint -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>

  <!-- Second link -->
  <link name="link2">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Second joint -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
</robot>
"""


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

def make_protobuf_channel(topic, class_type):
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

      # # Send the model data only when a client subscribes, to save bandwidth
      # if channel_id == scene_id:
      #     scene_update = SceneUpdate()
      #     entity = scene_update.entities.add()
      #     entity.timestamp.FromNanoseconds(now)
      #     entity.id = "model"
      #     entity.frame_id = "model"
      #     entity.frame_locked = True  # allow the entity to move when we update the "model" frame transforms
      #     model = entity.models.add()
      #     model.pose.position.x = 0
      #     model.pose.position.y = 0
      #     model.pose.position.z = 0
      #     q = Quaternion(axis=[0, 0, 1], angle=0.25 * 2 * math.pi * now / 1e9)
      #     model.pose.orientation.x = q.x
      #     model.pose.orientation.y = q.y
      #     model.pose.orientation.z = q.z
      #     model.pose.orientation.w = q.w
      #     model.color.r = 0.6
      #     model.color.g = 0.2
      #     model.color.b = 1
      #     model.color.a = 0.8

      #     # Use scale.x/y/z = 1 to use the original scale factor embedded in the model
      #     model.scale.x = 0.01
      #     model.scale.y = 0.01
      #     model.scale.z = 0.01
      #     model.data = flamingo_data
      #     model.media_type = "model/gltf-binary"

      #     asyncio.create_task(
      #         server.send_message(
      #             scene_chan_id, now, scene_update.SerializeToString()
      #         )
      #     )

    async def on_unsubscribe(self, server, channel_id):
      print(f"{Fore.RED}Last client unsubscribed from: {channel_id}{Fore.RESET}")


  async with FoxgloveServer("0.0.0.0", 8765, "example server") as server:
    server.set_listener(Listener())

    # def make_protobuf_channel(server, topic, msg_class):
    #   info = make_channel_info(topic, msg_class)
    #   channel_id = await server.add_channel( info )
    #   return channel_id

    # chan_id = await server.add_channel( make_channel_info("lidar_msg", LaserScan) )
    # image_id = await server.add_channel( make_channel_info("image_msg", RawImage) )
    # cal_id = await server.add_channel( make_channel_info("cal_msg", CameraCalibration) )
    # tf_id = await server.add_channel( make_channel_info("tf_msg", FrameTransform) )
    # imu_id = await server.add_channel( make_channel_info("imu_msg", Imu) )
    scene_id = await server.add_channel( make_protobuf_channel("scene_msg", SceneUpdate) )
    # scene_id = make_protobuf_channel(server,"scene_msg", SceneUpdate)

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
      cube.size.x = 1
      cube.size.y = 1
      cube.size.z = 1
      cube.pose.position.x = 2
      cube.pose.position.y = 0
      cube.pose.position.z = 0
      q = Quaternion(axis=[0, 1, 1], angle=i * 0.1)
      cube.pose.orientation.x = q.x
      cube.pose.orientation.y = q.y
      cube.pose.orientation.z = q.z
      cube.pose.orientation.w = q.w
      # cube.pose.orientation.x = 0
      # cube.pose.orientation.y = 0
      # cube.pose.orientation.z = 0
      # cube.pose.orientation.w = 1
      cube.color.r = 0.6
      cube.color.g = 0.2
      cube.color.b = 1
      cube.color.a = 1

      await server.send_message(scene_id, now, scene_update.SerializeToString())


if __name__ == "__main__":
  # camera_thread = Thread(target=camera_worker, args=("hello",))
  # camera_thread.start()

  run_cancellable(main())

  # gRun = False
  # camera_thread.join()