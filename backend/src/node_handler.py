import time
import asyncio
from datetime import datetime, timezone
import gs_interfaces.msg
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from loguru import logger
from rosidl_runtime_py import message_to_ordereddict
from fastapi import WebSocket, APIRouter, Query, HTTPException
from database.influx_client import (
    InfluxNotAvailableException,
    BucketNotFoundException,
    BadQueryException,
)

# The time after we mark topic as dead / inactive
TIMEOUT_THRESHOLD = 5

class NodeHandler(Node):

    def __init__(self, msg_config, app_loop=None, write_queue=None, influx_client=None):
        self.load_config(msg_config)
        super().__init__(self.msg_type)

        self.router = APIRouter()
        self.router.add_api_websocket_route(
            f"/{self.topic_name}", self.websocket_endpoint)
        self.router.add_api_route(
            f"/{self.topic_name}/query", self.query, methods=["GET"])

        # FastAPI asyncio loop where websocket broadcasts will run
        self.app_loop = app_loop

        # Shared queue to hand off messages for Influx writes
        self.write_queue = write_queue

        self.ic = influx_client

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1      # tylko 1 ostatnia wiadomość w kolejce
        )
        self.subscription = self.create_subscription(
            getattr(gs_interfaces.msg, self.msg_type), self.topic_name, self.msg_callback, qos)

        self.connected_clients = set()
        self.curr_msg = None

    def msg_callback(self, msg):
        now = datetime.now(timezone.utc)
        ros_time = Time()
        ros_time.sec = int(now.timestamp())                     # sekundy od epoki
        ros_time.nanosec = now.microsecond * 1000               # mikrosekundy -> nanosekundy
        msg.header.stamp = ros_time

        self.curr_msg = message_to_ordereddict(msg)

        # Schedule websocket broadcast on FastAPI loop (non-blocking)
        if self.app_loop is not None:
            try:
                asyncio.run_coroutine_threadsafe(self.broadcast_message(self.curr_msg), self.app_loop)
            except Exception as e:
                logger.warning(f"Failed to schedule broadcast on app loop: {e}")

        # Enqueue for Influx writing (writer thread will call insert_data)
        if self.write_queue is not None:
            try:
                # Put tuple (InfluxClient instance, message dict)
                self.write_queue.put_nowait((self.msg_type, self.curr_msg))
            except Exception as e:
                logger.warning(f"Write queue put failed: {e}")

    async def websocket_endpoint(self, ws: WebSocket):
            await ws.accept()
            self.connected_clients.add(ws)
            logger.info(f"Client connected to {self.topic_name}. Total: {len(self.connected_clients)}")
            try:
                while True:
                    await asyncio.sleep(1) 
                    
                    if self.curr_msg:
                        try:
                            stamp_sec = int(self.curr_msg.get('header', {}).get('stamp', {}).get('sec', 0))
                            if (time.time() - stamp_sec) > TIMEOUT_THRESHOLD:
                                await ws.send_json({"status": "timeout", "msg": "Topic inactive"})
                        except Exception:
                            pass
            except Exception as e:
                logger.debug(f"WebSocket on {self.topic_name} closed: {e}")
            finally:
                if ws in self.connected_clients:
                    self.connected_clients.remove(ws)

    async def broadcast_message(self, message):
        for client in list(self.connected_clients):
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
        self.connected_clients.clear()

    async def query(
        self,
        field_name: str = Query(..., description="Field key to query"),
        time_range: int = Query(1, ge=1, le=10),
    ):
        # Keep existing query behavior (uses Flux or refactor to InfluxQL in generator)
        flux_query = f'''
        from(bucket: "{self.ic.bucket}")
          |> range(start: -{time_range}m)
          |> filter(fn: (r) => r._measurement == "{self.msg_type}")
          |> filter(fn: (r) => r._field == "{field_name}")
          |> aggregateWindow(every: 1s, fn: last, createEmpty: false)
          |> limit(n: {time_range * 60})
        '''
        try:
            records = self.ic.query_data(flux_query)
            return {"records": records}
        except (
            InfluxNotAvailableException,
            BucketNotFoundException,
            BadQueryException,
        ) as e:
            raise HTTPException(status_code=e.STATUS_CODE, detail=e.DESCRIPTION)