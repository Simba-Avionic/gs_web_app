import rclpy
import json
import signal
import sys
import os
import threading

from time import sleep
from loguru import logger
from fastapi import FastAPI
from contextlib import asynccontextmanager
from fastapi import HTTPException
from fastapi.responses import JSONResponse, FileResponse
from fastapi.middleware.cors import CORSMiddleware
from src.node_handler import NodeHandler
from src.server_telemetry import ServerTelemetry
# from mavlink import MavlinkClient
from rclpy.executors import MultiThreadedExecutor
from dotenv import dotenv_values, find_dotenv

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from shared.paths import CONFIG_JSON_PATH, TILES_DIRECTORY

CONFIG = None
TOPICS = []  # Global variable to keep track of NodeHandler instances
env_values = dotenv_values(find_dotenv())

def shutdown():
    global TOPICS
    for topic in TOPICS:
        topic.stop()

    if rclpy.ok():
        try:
            rclpy.shutdown()
        except Exception as e:
            logger.error(f"Error during ROS2 shutdown: {e}")

def load_main_config(path_to_conf):
    global CONFIG
    with open(path_to_conf, "r") as f:
        CONFIG = json.load(f)

def signal_handler(sig, frame):
    logger.info('SIGINT received, shutting down...')
    shutdown()
    sys.exit(0)

@asynccontextmanager
async def lifespan(app: FastAPI):
    rclpy.init()
    load_main_config(CONFIG_JSON_PATH)
    executor = MultiThreadedExecutor()

    global TOPICS
    TOPICS = []

    try:
        st = ServerTelemetry()
        app.include_router(st.router)
        logger.info("ServerTelemetry initialized successfully!")
    except Exception as e:
        logger.error(f"Error when initializing server telemetry endpoint: {e}")

    for msg in CONFIG["topics"]:
        try:
            nh = NodeHandler(msg)
            executor.add_node(nh)
            app.include_router(nh.router)
            TOPICS.append(nh)
            logger.info(f"{msg['msg_type']} NodeHandler initialized successfully!")
        except Exception as e:
            logger.warning(f"Couldn't create/start NodeHandler for {msg['msg_type']}: {e}")
        sleep(0.05)

    stop_event = threading.Event()
    def executor_spin():
        try:
            executor.spin()
        except Exception as e:
            logger.error(f"Executor stopped unexpectedly: {e}")
        finally:
            stop_event.set()
    
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    yield

    # executor.shutdown()
    stop_event.wait(timeout=5)

    for nh in TOPICS:
        try:
            nh.destroy_node()
        except Exception as e:
            logger.warning(e)
    
    shutdown()
    logger.info("Shutdown complete.")

app = FastAPI(lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/tiles/{layer}/{z}/{x}/{y}.png")
def get_tile(layer: str, z: int, x: int, y: int):
    tile_path = os.path.join(TILES_DIRECTORY, layer, str(z), str(x), f"{y}.png")
    if not os.path.isfile(tile_path):
        raise HTTPException(status_code=404, detail="Tile not found")

    return FileResponse(tile_path, media_type="image/png")

@app.get("/config")
async def get_config():
    return JSONResponse(content=CONFIG)

if __name__ == "__main__":
    # signal.signal(signal.SIGINT, signal_handler)
    import uvicorn, argparse

    parser = argparse.ArgumentParser(description="Start the Uvicorn server.")
    parser.add_argument('--host', type=str, default=env_values.get("IP_ADDRESS"), help='Host address of the server')
    parser.add_argument('--port', type=int, default=8000, help='Port number of the server')
    args = parser.parse_args()

    try:
        uvicorn.run(app, host=args.host, port=args.port)
    except Exception as e:
        logger.error(f"Error running Uvicorn: {e}")
