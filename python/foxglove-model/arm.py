#!/usr/bin/env python3

###########################
#
# This DOESN'T WORK!!!!
#
###########################

import asyncio
import websockets
import json
import time
import math

# Function to create a SceneUpdate message for the robot arm
def create_scene_update(timestamp, joint1_angle, joint2_angle):
    # Base link (a cube)
    base = {
        "pose": {
            "position": {"x": 0, "y": 0, "z": 0},
            "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
        },
        "size": {"x": 0.1, "y": 0.1, "z": 0.1},
        "color": {"r": 0.5, "g": 0.5, "b": 0.5, "a": 1}
    }

    # Link 1 (cylinder) - rotates around Z-axis at base
    link1 = {
        "pose": {
            "position": {"x": 0, "y": 0, "z": 0.15},  # Offset from base
            "orientation": {
                "x": 0,
                "y": 0,
                "z": math.sin(joint1_angle / 2),
                "w": math.cos(joint1_angle / 2)
            }
        },
        "size": {"x": 0.02, "y": 0.02, "z": 0.3},  # Thin cylinder
        "color": {"r": 0, "g": 0, "b": 1, "a": 1}
    }

    # Link 2 (cylinder) - attached to end of Link 1, rotates around Z-axis
    link2 = {
        "pose": {
            "position": {
                "x": 0.3 * math.cos(joint1_angle),  # End of Link 1
                "y": 0.3 * math.sin(joint1_angle),
                "z": 0.15
            },
            "orientation": {
                "x": 0,
                "y": 0,
                "z": math.sin((joint1_angle + joint2_angle) / 2),
                "w": math.cos((joint1_angle + joint2_angle) / 2)
            }
        },
        "size": {"x": 0.02, "y": 0.02, "z": 0.3},
        "color": {"r": 1, "g": 0, "b": 0, "a": 1}
    }

    # SceneUpdate message
    scene_update = {
        "deletions": [],  # Clear previous entities if needed
        "entities": [
            {"timestamp": timestamp, "type": "cube", "cube": base},
            {"timestamp": timestamp, "type": "cylinder", "cylinder": link1},
            {"timestamp": timestamp, "type": "cylinder", "cylinder": link2}
        ]
    }
    return scene_update

# WebSocket server to send data to Foxglove
async def send_robot_data(websocket):
    start_time = time.time()
    while True:
        # Simulate joint motion
        elapsed_time = time.time() - start_time
        joint1_angle = math.sin(elapsed_time)
        joint2_angle = math.cos(elapsed_time)
        timestamp = {"sec": int(time.time()), "nsec": int((time.time() % 1) * 1e9)}

        # Create and serialize the SceneUpdate message
        scene_update = create_scene_update(timestamp, joint1_angle, joint2_angle)
        message = json.dumps({
            "topic": "/robot_arm",
            "schemaName": "foxglove.SceneUpdate",
            "message": scene_update
        })

        # Send the message over WebSocket
        await websocket.send(message)
        await asyncio.sleep(0.1)  # Update at 10 Hz

# Start the WebSocket server
async def main():
    server = await websockets.serve(send_robot_data, "localhost", 8765)
    print("WebSocket server running on ws://localhost:8765")
    await server.wait_closed()

if __name__ == "__main__":
    asyncio.run(main())