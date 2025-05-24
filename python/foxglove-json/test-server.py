#!/usr/bin/env python3
import asyncio
import json
import time
from foxglove_websocket import run_cancellable
from foxglove_websocket.server import FoxgloveServer

async def main():
    # Initialize the server
    async with FoxgloveServer("0.0.0.0", 8765, "example server") as server:
        # Add a channel for JSON messages
        channel_id = await server.add_channel({
            "topic": "example_msg",
            "encoding": "json",
            "schemaName": "ExampleMsg",
            "schema": json.dumps({
                "type": "object",
                "properties": {
                    "msg": {"type": "string"},
                    "count": {"type": "number"}
                }
            })
        })

        # Send messages periodically
        count = 0
        while True:
            count += 1
            message = json.dumps({"msg": "Hello!", "count": count}).encode("utf8")
            await server.send_message(channel_id, time.time_ns(), message)
            await asyncio.sleep(0.2)

if __name__ == "__main__":
    run_cancellable(main())