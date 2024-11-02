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
from rclpy.executors import MultiThreadedExecutor
from dotenv import dotenv_values, find_dotenv

MAIN_FILE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG = None
CONFIG_PATH = os.path.join(MAIN_FILE_DIR, "../config.json")
TOPICS = []  # Global variable to keep track of NodeHandler instances
env_values = dotenv_values(find_dotenv())

def shutdown():
    global TOPICS
    for topic in TOPICS:
        topic.stop()  # Ensure all topic handlers are stopped

    try:
        rclpy.shutdown()  # Shut down ROS2
    except Exception as e:
        logger.error(f"Error during ROS2 shutdown: {e}")

def load_main_config(path_to_conf):
    global CONFIG
    with open(path_to_conf, "r") as f:
        CONFIG = json.load(f)

@asynccontextmanager
async def lifespan(app: FastAPI):
    rclpy.init()
    load_main_config(CONFIG_PATH)
    executor = MultiThreadedExecutor()

    global TOPICS
    TOPICS = []

    try:
        st = ServerTelemetry()
        app.include_router(st.router)
        logger.info("ServerTelemetry initialized successfully!")
    except Exception as e:
        logger.error(f"Error when initializing server telemetry endpoint! {e}")

    for msg in CONFIG["topics"]:
        try:
            nh = NodeHandler(msg)
            executor.add_node(nh)
            app.include_router(nh.router)
            TOPICS.append(nh)
            logger.info(f"{msg['msg_type']} NodeHandler initialized successfully!")
        except Exception as e:
            logger.warning(f"Couldn't create/start NodeHandler for {msg['msg_type']}: {e}")
        sleep(0.1)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    yield
    executor.shutdown()
    for nh in TOPICS:
        try:
            nh.destroy_node()
        except Exception as e:
            logger.warning(e)
    shutdown()

app = FastAPI(lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

TILES_DIRECTORY = "../app/public/tiles"

@app.get("/tiles/{layer}/{z}/{x}/{y}.png")
def get_tile(layer: str, z: int, x: int, y: int):
    tile_path = os.path.join(TILES_DIRECTORY, layer, str(z), str(x), f"{y}.png")
    # logger.debug(tile_path)
    
    if not os.path.isfile(tile_path):
        raise HTTPException(status_code=404, detail="Tile not found")

    return FileResponse(tile_path, media_type="image/png")

@app.get("/config")
async def get_config():
    return JSONResponse(content=CONFIG)

def signal_handler(sig, frame):
    logger.info('SIGINT received, shutting down...')
    shutdown()
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    import uvicorn, argparse

    parser = argparse.ArgumentParser(description="Start the Uvicorn server.")
    parser.add_argument('--host', type=str, default=env_values.get("IP_ADDRESS"), help='Host address of the server')
    parser.add_argument('--port', type=int, default=8000, help='Port number of the server')
    args = parser.parse_args()

    try:
        uvicorn.run(app, host=args.host, port=args.port)
    except KeyboardInterrupt:
        logger.info('KeyboardInterrupt received, shutting down...')
        shutdown()
