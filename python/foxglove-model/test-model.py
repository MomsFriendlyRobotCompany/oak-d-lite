#!/usr/bin/env python3

import asyncio
import math
import requests
import time
from base64 import b64encode
from foxglove_websocket import run_cancellable
from foxglove_websocket.server import FoxgloveServer, FoxgloveServerListener
from foxglove_schemas_protobuf.SceneUpdate_pb2 import SceneUpdate
from foxglove_schemas_protobuf.FrameTransform_pb2 import FrameTransform
from mcap_protobuf.schema import build_file_descriptor_set
from pyquaternion import Quaternion


async def main():
    flamingo_data = requests.get(
        "https://github.com/mrdoob/three.js/raw/dev/examples/models/gltf/Flamingo.glb"
    ).content

    async with FoxgloveServer("0.0.0.0", 8765, "example server") as server:
        scene_chan_id = await server.add_channel(
            {
                "topic": "scene",
                "encoding": "protobuf",
                "schemaName": SceneUpdate.DESCRIPTOR.full_name,
                "schema": b64encode(
                    build_file_descriptor_set(SceneUpdate).SerializeToString()
                ).decode("ascii"),
            }
        )
        tf_chan_id = await server.add_channel(
            {
                "topic": "transforms",
                "encoding": "protobuf",
                "schemaName": FrameTransform.DESCRIPTOR.full_name,
                "schema": b64encode(
                    build_file_descriptor_set(FrameTransform).SerializeToString()
                ).decode("ascii"),
            }
        )

        class Listener(FoxgloveServerListener):
            def on_subscribe(self, server, channel_id):
                # Send the model data only when a client subscribes, to save bandwidth
                if channel_id == scene_chan_id:
                    scene_update = SceneUpdate()
                    entity = scene_update.entities.add()
                    entity.timestamp.FromNanoseconds(now)
                    entity.id = "model"
                    entity.frame_id = "model"
                    entity.frame_locked = True  # allow the entity to move when we update the "model" frame transforms
                    model = entity.models.add()
                    model.pose.position.x = 0
                    model.pose.position.y = 0
                    model.pose.position.z = 0
                    q = Quaternion(axis=[0, 0, 1], angle=0.25 * 2 * math.pi * now / 1e9)
                    model.pose.orientation.x = q.x
                    model.pose.orientation.y = q.y
                    model.pose.orientation.z = q.z
                    model.pose.orientation.w = q.w
                    model.color.r = 0.6
                    model.color.g = 0.2
                    model.color.b = 1
                    model.color.a = 0.8

                    # Use scale.x/y/z = 1 to use the original scale factor embedded in the model
                    model.scale.x = 0.01
                    model.scale.y = 0.01
                    model.scale.z = 0.01
                    model.data = flamingo_data
                    model.media_type = "model/gltf-binary"

                    asyncio.create_task(
                        server.send_message(
                            scene_chan_id, now, scene_update.SerializeToString()
                        )
                    )

            def on_unsubscribe(self, server, channel_id):
                print(f">>> {server} unsubscribed from {channel_id}")

        server.set_listener(Listener())

        # Send the FrameTransform every frame to update the model's position
        while True:
            await asyncio.sleep(0.02)
            now = time.time_ns()

            transform = FrameTransform()
            transform.parent_frame_id = "root"
            transform.child_frame_id = "model"
            transform.timestamp.FromNanoseconds(now)
            transform.translation.x = 0
            transform.translation.y = 0
            transform.translation.z = math.sin(now / 1e9 * 0.5)
            q = Quaternion(axis=[0, 0, 1], angle=-0.1 * 2 * math.pi * now / 1e9)
            transform.rotation.x = q.x
            transform.rotation.y = q.y
            transform.rotation.z = q.z
            transform.rotation.w = q.w
            await server.send_message(tf_chan_id, now, transform.SerializeToString())


if __name__ == "__main__":
    run_cancellable(main())