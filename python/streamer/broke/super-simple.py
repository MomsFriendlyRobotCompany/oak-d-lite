#!/usr/bin/env python3

# BROKEN


import json
import time
from websocket import create_connection

# Simple URDF string (a basic single-link robot for demonstration)
urdf_example = """
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>
</robot>
"""

# Foxglove WebSocket server (default local address for Foxglove Studio)
ws_url = "ws://localhost:8765"

# Connect to Foxglove WebSocket
ws = create_connection(ws_url)

# Advertise the URDF topic
advertise_msg = {
    "op": "advertise",
    "channel": "/robot_description",
    "type": "std_msgs/String",
    "encoding": "json"
}
ws.send(json.dumps(advertise_msg))

# Publish the URDF data
publish_msg = {
    "op": "publish",
    "channel": "/robot_description",
    "data": json.dumps({"data": urdf_example})
}

try:
    print("Sending URDF to Foxglove...")
    while True:
        ws.send(json.dumps(publish_msg))
        print("Published URDF to /robot_description")
        time.sleep(1)  # Send every second to keep it alive
except KeyboardInterrupt:
    print("Stopping...")
finally:
    # Clean up by closing the connection
    ws.close()

print("Done!")