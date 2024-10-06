import time
import asyncio
import threading
from loguru import logger
from rclpy import executors
from fastapi import WebSocket, APIRouter, Request, HTTPException
from src.topic_receiver import TopicReceiver
from database.influx_client import (
    InfluxClient,
    InfluxNotAvailableException,
    BucketNotFoundException,
    BadQueryException,
)


class NodeHandler:

    def __init__(self, msg_config):
        # Load configuration and initialize ROS receiver
        self.load_config(msg_config)
        self.receiver = TopicReceiver(
            self.msg_type, self.topic_name, self.interval, self.msg_fields)

        # Router setup for WebSocket and HTTP endpoints
        self.router = APIRouter()
        self.router.add_api_websocket_route(f"/{self.topic_name}", self.return_msg)
        self.router.add_api_websocket_route(f"/{self.topic_name}/query", self.query)

        # Executor setup for ROS2 communication
        self.executor = executors.MultiThreadedExecutor()
        self.executor.add_node(self.receiver)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        # InfluxDB client setup and background data insertion thread
        self.ic = InfluxClient(self.msg_type, self.topic_name, self.msg_fields)
        if self.ic:
            self.db_insert = threading.Thread(target=self.db_thread)
            self.db_insert.start()

        self.connected_clients = set()

    async def return_msg(self, ws: WebSocket):
        await ws.accept()
        self.connected_clients.add(ws)
        logger.info(f"New WebSocket client connected: {len(self.connected_clients)} clients connected.")

        try:
            last_msg = None
            while True:
                await asyncio.sleep(self.interval / 1000)
                data = self.receiver.get_msg()
                if data:
                    last_msg = data
                    await ws.send_json(data)
                elif last_msg and (time.time() - int(last_msg['header']['stamp']['sec'])) > (self.interval / 100):
                    await ws.send_json(None)
        except Exception as e:
            logger.error(f"WebSocket connection closed: {e}")
        finally:
            self.connected_clients.remove(ws)
            logger.info(f"WebSocket client disconnected: {len(self.connected_clients)} clients connected.")
            await ws.close()

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
            logger.info(f"Removed disconnected client. {len(self.connected_clients)} clients remaining.")

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

    def db_thread(self):
        """
        Background thread to handle database operations.
        """
        while True:
            data = self.receiver.get_msg_db()
            if data:
                self.ic.insert_data(data)
            time.sleep(self.interval / 100)

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
