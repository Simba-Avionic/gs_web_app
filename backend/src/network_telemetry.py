import asyncio
from loguru import logger
from fastapi import WebSocket, APIRouter

NETWORK_DEVICES = [
    {"id": "rpi_lp", "name": "Launchbox", "location": "Launchpad", "ip": "192.168.10.102", "icon": "statuses/svg/launchbox_icon.svg"},
    {"id": "router_mc", "name": "Router Mikrotik", "location": "Mission Control", "ip": "192.168.10.121", "icon": "statuses/svg/router_icon.svg"},
    {"id": "router_lp", "name": "Router Mikrotik", "location": "Launchpad", "ip": "192.168.10.122", "icon": "statuses/svg/router_icon.svg"},
    {"id": "switch_lp", "name": "Switch", "location": "Launchpad", "ip": "192.168.10.161", "icon": "statuses/svg/router_icon.svg"},
    {"id": "ant_mc", "name": "Antena", "location": "Mission Control", "ip": "192.168.10.141", "icon": "statuses/svg/antenna_icon.svg"},
    {"id": "ant_lp", "name": "Antena", "location": "Launchpad", "ip": "192.168.10.142", "icon": "statuses/svg/antenna_icon.svg"},
    {"id": "cam1_lp", "name": "Camera1", "location": "Launchpad", "ip": "192.168.10.151", "icon": "statuses/svg/camera_icon.svg"},
    {"id": "cam2_lp", "name": "Camera2", "location": "Launchpad", "ip": "192.168.10.152", "icon": "statuses/svg/camera_icon.svg"},
]

class NetworkTelemetry:

    def __init__(self, influx_client=None) -> None:
        
        self.topic_name = "/network/telemetry"
        self.msg_name = "NetworkHeartbeat" 
        self.msg_fields = """
        [
            {
                "val_name": "is_online",
                "type": "boolean",
                "unit": "status"
            }
        ]
        """

        self.router = APIRouter()
        self.router.add_api_websocket_route(self.topic_name, self.websocket_endpoint)
        self.ic = influx_client

        self.connected_clients = set()
        self.stop_event = asyncio.Event()
        asyncio.create_task(self.start_sending_data())

    async def ping_ip(self, ip: str) -> bool:
        """
        Asynchronously pings an IP address. 
        Note: The arguments '-c 1' and '-W 1' are for Linux. 
        If running on Windows, use '-n 1' and '-w 1000' instead.
        """
        try:
            # -c 1: send exactly 1 ping packet
            # -W 1: wait exactly 1 second for a response before timing out
            process = await asyncio.create_subprocess_exec(
                "ping", "-c", "1", "-W", "1", ip,
                stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.DEVNULL
            )
            await process.wait()
            # returncode 0 means the ping was successful
            return process.returncode == 0
        except Exception as e:
            logger.error(f"Error pinging {ip}: {e}")
            return False

    async def get_network_health(self):
        ping_tasks = [self.ping_ip(device["ip"]) for device in NETWORK_DEVICES]
        results = await asyncio.gather(*ping_tasks)
        
        health_data = {}
        for device, is_online in zip(NETWORK_DEVICES, results):
            health_data[device["id"]] = {
                "name": device["name"],
                "location": device["location"],
                "ip": device["ip"],
                "icon": device.get("icon", "statuses/svg/default_icon.svg"), 
                "is_online": is_online
            }
        
        return health_data

    async def start_sending_data(self):
        """
        A background task that continuously gathers and sends telemetry data to connected clients.
        """
        while not self.stop_event.is_set():
            # Ping every 3 seconds (adjust as needed, pings can be noisy on the network)
            await asyncio.sleep(3) 
            
            # Only ping if we actually have clients listening to save CPU/Network bandwidth
            if self.connected_clients:
                data = await self.get_network_health()
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
                # Keep the connection alive
                await asyncio.sleep(1)                
        except Exception as e:
            logger.error(f"WebSocket connection closed: {e}")
        finally:
            await self.handle_client_disconnection(ws)

    async def broadcast_message(self, message):
        """
        Broadcasts a message to all currently connected WebSocket clients.
        """
        for client in list(self.connected_clients):  
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
        self.stop_event.set()
        logger.info("NetworkTelemetry stopped.")