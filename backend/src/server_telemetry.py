import psutil
import asyncio
from loguru import logger
from fastapi import WebSocket, APIRouter, Request, HTTPException
from database.influx_client import (
    InfluxNotAvailableException,
    BucketNotFoundException,
    BadQueryException,
)

class ServerTelemetry:

    def __init__(self, influx_client) -> None:
        
        self.topic_name = "/server/telemetry"
        self.msg_name = "ServerTelemetry" 
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
            }
        ]
        """

        self.router = APIRouter()
        self.router.add_api_websocket_route(self.topic_name, self.websocket_endpoint)
        # self.router.add_api_websocket_route(f"{self.topic_name}/query", self.query)
        self.ic = influx_client

        self.connected_clients = set()
        self.stop_event = asyncio.Event()
        asyncio.create_task(self.start_sending_data())

    def get_system_data(self):
        cpu_usage = psutil.cpu_percent()
        memory_info = psutil.virtual_memory()
        memory_usage = memory_info.percent
        disk_info = psutil.disk_usage('/')
        disk_usage = disk_info.percent

        cpu_temp = None
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                cpu_temp = int(f.read()) / 1000.0
        except Exception:
            cpu_temp = None

        return {
            "cpu_usage": cpu_usage,
            "memory_usage": memory_usage,
            "disk_usage": disk_usage,
            "cpu_temperature": cpu_temp,
        }

    async def start_sending_data(self):
        """
        A background task that continuously gathers and sends telemetry data to connected clients.
        TODO: insert data to DB
        """
        while not self.stop_event.is_set():
            await asyncio.sleep(1)
            data = self.get_system_data()
            if data:
                await self.broadcast_message(data)

    async def websocket_endpoint(self, ws: WebSocket):
        """
        WebSocket endpoint to handle messages for each client and broadcast messages.
        """
        await ws.accept()
        self.connected_clients.add(ws)
        # logger.info(f"New WebSocket client {ws} connected: {len(self.connected_clients)} clients connected.")
        try:
            while True:
                await asyncio.sleep(1)                
        except Exception as e:
            logger.error(f"WebSocket connection closed: {e}")
        finally:
            await self.connected_clients.remove(ws)

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

    def stop(self):
        logger.info(f"ServerTelemetry stopped.")