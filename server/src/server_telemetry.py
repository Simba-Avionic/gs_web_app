import psutil
import time
import asyncio
import threading
from loguru import logger
from fastapi import WebSocket, APIRouter, Request, HTTPException
from database.influx_client import (
    InfluxClient,
    InfluxNotAvailableException,
    BucketNotFoundException,
    BadQueryException,
)

class ServerTelemetry:

    def __init__(self) -> None:
        
        self.topic_name = "/server/telemetry"
        self.msg_name = "ServerTelemetry" 
        self.interval = 1000
        self.msg_fields = """
        [
            {
                "val_name": "cpu_usage",
                "type": "float32",
                "unit": "%"
            },
            {
                "val_name": "memory_usage",
                "type": "float32",
                "unit": "%"
            },
            {
                "val_name": "cpu_temperature",
                "type": "float32",
                "unit": "°C"
            },
            {
                "val_name": "load_1_min",
                "type": "float32",
            },
            {
                "val_name": "load_5_min",
                "type": "float32",
            },
            {
                "val_name": "load_15_min",
                "type": "float32",
            }
        ]
        """

        self.router = APIRouter()
        self.router.add_api_websocket_route(self.topic_name, self.return_msg)
        self.router.add_api_websocket_route(f"{self.topic_name}/query", self.query)
        self.ic = InfluxClient(self.msg_name, self.topic_name, self.msg_fields)

        self.connected_clients = set()

    def get_system_data(self):
        cpu_usage = psutil.cpu_percent()

        memory_info = psutil.virtual_memory()
        memory_usage = memory_info.percent

        disk_info = psutil.disk_usage('/')
        disk_usage = disk_info.percent

        try:
            temp = psutil.sensors_temperatures()
            cpu_temp = temp['cpu_thermal'][0].current if 'cpu_thermal' in temp else None
        except Exception:
            cpu_temp = None

        load_1, load_5, load_15 = psutil.getloadavg()

        return {
            "cpu_usage": cpu_usage,
            "memory_usage": memory_usage,
            "disk_usage": disk_usage,
            "cpu_temperature": cpu_temp,
            "load_1_min": load_1,
            "load_5_min": load_5,
            "load_15_min": load_15
        }

    async def return_msg(self, ws: WebSocket):
        """
        WebSocket endpoint to handle messages for each client and broadcast messages.
        """
        await ws.accept()
        self.connected_clients.add(ws)
        # logger.info(f"New WebSocket client {ws} connected: {len(self.connected_clients)} clients connected.")
        last_time = 0

        try:
            last_msg = None
            while True:
                await asyncio.sleep(self.interval / 1000)
                data = self.get_system_data()
                if data:
                    last_msg = data
                    await self.broadcast_message(data)
                elif last_msg and (time.time() - int(last_msg['header']['stamp']['sec'])) > (self.interval / 100):
                    await self.broadcast_message(None)
                # logger.info(self.connected_clients) 
                
        except Exception as e:
            logger.error(f"WebSocket connection closed: {e}")
        finally:
            if ws in self.connected_clients:
                self.connected_clients.remove(ws)
                # logger.info(f"WebSocket client disconnected: {len(self.connected_clients)} clients connected.")
                await ws.close()

    async def broadcast_message(self, message):
        """
        Broadcasts a message to all currently connected WebSocket clients.
        """
        for client in list(self.connected_clients):  # Convert set to list to avoid runtime modification issues
            try:
                await client.send_json(message)
            except Exception as e:
                logger.error(f"Failed to send message to a client: {e}")
                await self.handle_client_disconnection(client)

    async def handle_client_disconnection(self, client: WebSocket):
        """
        Handles client disconnection scenarios to safely remove from the connected clients set.
        """
        if client in self.connected_clients:
            self.connected_clients.remove(client)
            logger.info(f"Removed disconnected client. {len(self.connected_clients)} clients remaining.")

    async def query(self, r: Request, time_range: int = 5):
        """
        HTTP endpoint to query historical data from the InfluxDB.
        """
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

    def stop(self):
        # self.connected_clients.clear()
        # self.db_insert.join(timeout=3)
        logger.info(f"ServerTelemetry stopped.")