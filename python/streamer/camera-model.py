#!/usr/bin/env python3
import asyncio
import requests
# from foxglove_websocket import run_cancellable
# from foxglove_websocket.server import FoxgloveServer
from foxglove_schemas_protobuf.SceneUpdate_pb2 import SceneUpdate
from foxglove_schemas_protobuf.ModelPrimitive_pb2 import ModelPrimitive
from foxglove_schemas_protobuf.FrameTransform_pb2 import FrameTransform
# from mcap_protobuf.schema import build_file_descriptor_set
from thebrian import build_file_descriptor_set, make_protobuf_channel
# from thebrian import STLModel as CameraModel
from thebrian import make_oak_model, make_tf
from base64 import b64encode
from google.protobuf.timestamp_pb2 import Timestamp
from foxglove_websocket import run_cancellable
from foxglove_websocket.server import FoxgloveServerListener
from foxglove_websocket.server import FoxgloveServer
import numpy as np
from colorama import Fore
import time
from squaternion import Quaternion
from gecko_messages import vector_t
# from gecko_messages import TransformStamped
# from gecko_messages import FrameTransform


class CameraListener(FoxgloveServerListener):
  channel_ids = {}
  async def on_subscribe(self, server, channel_id):
    print(f"{Fore.GREEN}First client subscribed to: {channel_id}{Fore.RESET}")

    # Send the model data only when a client subscribes, to save bandwidth
    if channel_id == self.channel_ids["scene"]:
      # scene_update = cam_model.get_scene(file_path)
      scene_update = camera_model

      now = time.time_ns()

      asyncio.create_task(
        server.send_message(channel_id, now, scene_update.SerializeToString())
      )
      print(f"{Fore.CYAN}/// Sent model data ///{Fore.RESET}")

  async def on_unsubscribe(self, server, channel_id):
    print(f"{Fore.RED}Last client unsubscribed from: {channel_id}{Fore.RESET}")


file_path = "../../docs/oak-d-lite/oak-d-lite.stl"
# cam_model = CameraModel()
camera_model = make_oak_model(file_path,"camera","root")

async def main():
    # Start Foxglove WebSocket server
    async with FoxgloveServer("0.0.0.0", 8765, "3D Model Example") as server:
      camlistener = CameraListener()
      server.set_listener(camlistener)

      scene_chan_id = await server.add_channel( make_protobuf_channel("scene", SceneUpdate) )
      tf_id = await server.add_channel( make_protobuf_channel("tf_msg", FrameTransform) )
      # tf_id = await server.add_channel( make_protobuf_channel("tf_msg", TransformStamped) )

      camlistener.channel_ids["scene"] = scene_chan_id
      camlistener.channel_ids["tf_msg"] = tf_id

      i = 0

      while True:
        await asyncio.sleep(0.01)

        i += 1

        now = time.time_ns()

        q = Quaternion.from_angle_axis(i*0.01, [1,0,1])
        # T = cam_model.get_tf(q=q, now=now)
        v = vector_t(0,0,1)
        T = make_tf("camera", "root",orientation=q, translation=v, time_ns=now)
        await server.send_message(tf_id, now, T.SerializeToString())

if __name__ == "__main__":
    run_cancellable(main())