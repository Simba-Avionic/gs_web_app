import time
import rclpy
import asyncio
import threading
import gs_interfaces.msg
from rclpy.node import Node
from loguru import logger
from rclpy import executors
from rosidl_runtime_py import message_to_ordereddict
from fastapi import WebSocket, APIRouter, Request, HTTPException
from database.influx_client import (
    InfluxClient,
    InfluxNotAvailableException,
    BucketNotFoundException,
    BadQueryException,
)


class NodeHandler(Node):

    def __init__(self, msg_config):
        # Load configuration and initialize ROS subscription
        self.load_config(msg_config)
        super().__init__(self.msg_type)

        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.run_event_loop, daemon=True).start()

        self.start_subscription_thread()
        # self.subscription = self.create_subscription(
        #     getattr(gs_interfaces.msg, self.msg_type), self.topic_name, self.msg_callback, 10)

        # Router setup for WebSocket and HTTP endpoints
        self.router = APIRouter()
        self.router.add_api_websocket_route(f"/{self.topic_name}", self.websocket_endpoint)
        # self.router.add_api_websocket_route(f"/{self.topic_name}/query", self.query)

        # Executor setup for ROS2 communication
        # self.executor = executors.MultiThreadedExecutor()
        # self.executor.add_node(self.subscription)
        # self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        # self.executor_thread.start()

        # InfluxDB client setup and background data insertion thread
        # self.ic = InfluxClient(self.msg_type, self.topic_name, self.msg_fields)
        # if self.ic:
        #     self.db_insert = threading.Thread(target=self.db_thread)
        #     self.db_insert.start()

        self.connected_clients = set()
        self.curr_msg = None


    def start_subscription_thread(self):
        subscription_thread = threading.Thread(target=self.create_subscription_and_spin)
        subscription_thread.daemon = True  # Ensure thread exits when the main program does
        subscription_thread.start()

    def create_subscription_and_spin(self):
        self.subscription = self.create_subscription(
            getattr(gs_interfaces.msg, self.msg_type), self.topic_name, self.msg_callback, 10)

    def msg_callback(self, msg):
        self.curr_msg = message_to_ordereddict(msg)
        # asyncio.run_coroutine_threadsafe(self.broadcast_message(self.curr_msg), self.loop)
        asyncio.create_task(self.broadcast_message(self.curr_msg))

    def run_event_loop(self):
        # Run the event loop
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    async def websocket_endpoint(self, ws: WebSocket):
        await ws.accept()
        self.connected_clients.add(ws)
        # logger.info(f"New WebSocket client connected: {len(self.connected_clients)} clients connected.")
        try:
            while True:
                await ws.receive_text() 
        except Exception as e:
            logger.error(f"WebSocket connection closed: {e}")
        finally:
            self.connected_clients.remove(ws)
            # logger.info(f"WebSocket client disconnected: {len(self.connected_clients)} clients connected.")
            # await ws.close()

    async def broadcast_message(self, message):
        for client in list(self.connected_clients):  # Convert set to list to avoid runtime modification issues
            try:
                await client.send_json(message)
            except Exception as e:
                logger.error(f"Failed to send message to a client: {e}")
                await self.handle_client_disconnection(client)

    async def handle_client_disconnection(self, client: WebSocket):
        if client in self.connected_clients:
            self.connected_clients.remove(client)
            # logger.info(f"Removed disconnected client. {len(self.connected_clients)} clients remaining.")

    def load_config(self, msg_config):
        self.msg_fields = msg_config["msg_fields"]
        self.msg_type = msg_config["msg_type"]
        self.topic_name = msg_config["topic_name"]
        self.interval = msg_config["interval"]

    def stop(self):
        self.executor.remove_node(self.receiver)
        self.receiver.destroy_node()
        self.executor.shutdown()
        self.executor_thread.join(timeout=5)
        # self.connected_clients.clear()
        # self.db_insert.join(timeout=3)
        logger.info(f"NodeHandler stopped for {self.receiver}")
