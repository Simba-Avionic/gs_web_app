# fastapi_websocket_ros2.py
import asyncio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
import gs_interfaces.msg
import json, logging
from uvicorn import Config, Server
from rosidl_runtime_py import message_to_ordereddict

app = FastAPI()

# Allow CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Update this for your frontend domain
    allow_methods=["*"],
    allow_headers=["*"],
)

clients = []

class ROS2Node(Node):
    def __init__(self):
        super().__init__('LoadCells')
        self.subscription = self.create_subscription(
            getattr(gs_interfaces.msg, "LoadCells"), 
            "tanking/load_cells", 
            self.telemetry_callback, 10)
        self.latest_data = None

    def telemetry_callback(self, msg):
        print(msg)
        self.latest_data = message_to_ordereddict(msg)
        asyncio.create_task(notify_clients(self.latest_data))

async def notify_clients(data):
    # Broadcast the latest telemetry data to all WebSocket clients
    logging.info(f"Notifying clients: {data}")
    for client in clients:
        await client.send_json(data)

# ROS2 thread to run the node
async def start_ros2_node():
    rclpy.init()
    ros_node = ROS2Node()

    # Running the node in an asyncio thread to avoid blocking the event loop
    try:
        while rclpy.ok():
            rclpy.spin_once(ros_node, timeout_sec=0.1)  # Non-blocking spin
            await asyncio.sleep(0.1)  # Give control back to the event loop
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    clients.append(websocket)
    logging.info(f"Client connected: {websocket.client}")  # Log client connection
    try:
        while True:
            await websocket.receive_text()  # Receive from the client (optional)
    except Exception as e:
        print(f"Client disconnected: {e}")
    finally:
        clients.remove(websocket)

async def main():
    config = Config(app=app, host="127.0.0.1", port=8000, log_level="info")
    server = Server(config)

    # Run the FastAPI server and ROS2 node concurrently
    ros2_task = asyncio.create_task(start_ros2_node())
    api_task = asyncio.create_task(server.serve())

    await asyncio.gather(ros2_task, api_task)

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    asyncio.run(main())
