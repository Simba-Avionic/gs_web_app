import time
import asyncio
import threading
import gs_interfaces.msg
from rclpy.node import Node
from loguru import logger
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
        self.load_config(msg_config)
        super().__init__(self.msg_type)

        self.router = APIRouter()
        self.router.add_api_websocket_route(f"/{self.topic_name}", self.websocket_endpoint)
        self.router.add_api_websocket_route(f"/{self.topic_name}/query", self.query)

        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.run_event_loop, daemon=True).start()

        self.start_subscription_thread()

        self.ic = InfluxClient(self.msg_type, self.topic_name, self.msg_fields)
        self.connected_clients = set()
        self.curr_msg = None

    def start_subscription_thread(self):
        self.subscription_thread = threading.Thread(target=self.create_subscription_and_spin)
        self.subscription_thread.daemon = True  # Ensure thread exits when the main program does
        self.subscription_thread.start()

    def create_subscription_and_spin(self):
        self.subscription = self.create_subscription(
            getattr(gs_interfaces.msg, self.msg_type), self.topic_name, self.msg_callback, 10)

    async def msg_callback(self, msg):
        self.curr_msg = message_to_ordereddict(msg)
        await self.broadcast_message(self.curr_msg)
        if self.ic.db_alive():
            self.ic.insert_data(self.curr_msg)

    def run_event_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    async def websocket_endpoint(self, ws: WebSocket):
        await ws.accept()
        self.connected_clients.add(ws)
        try:
            while True:
                await asyncio.sleep(self.interval / 1000)
                if self.curr_msg and (time.time() - int(self.curr_msg['header']['stamp']['sec'])) > (self.interval / 100):
                    await self.broadcast_message("None")
        except Exception as e:
            logger.error(f"WebSocket connection closed: {e}")
        finally:
            self.connected_clients.remove(ws)

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

    def load_config(self, msg_config):
        self.msg_fields = msg_config["msg_fields"]
        self.msg_type = msg_config["msg_type"]
        self.topic_name = msg_config["topic_name"]
        self.interval = msg_config["interval"]

    def stop(self):
        if self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)

        if self.subscription_thread is not None:
            self.subscription_thread.join(timeout=3)

        self.connected_clients.clear()

    async def query(self, r: Request, time_range: int = 5):
        try:
            records = []
            # records = await self.ic.query_(self.msg_type, some_value_name, time_range=time_range)
            return records  # Return the records, adjust as necessary for your use case
        except (
            InfluxNotAvailableException,
            BucketNotFoundException,
            BadQueryException,
        ) as e:
            raise HTTPException(
                status_code=e.STATUS_CODE,
                detail=e.DESCRIPTION,
            )