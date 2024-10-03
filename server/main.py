import rclpy
import json
import signal
import sys

from time import sleep
from loguru import logger
from fastapi import FastAPI
from contextlib import asynccontextmanager
from fastapi.responses import JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from src.node_handler import NodeHandler
from src.server_telemetry import ServerTelemetry

CONFIG = None
CONFIG_PATH = "../config.json"
TOPICS = []  # Global variable to keep track of NodeHandler instances

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

    global TOPICS
    TOPICS = []

    try:
        st = ServerTelemetry()
        app.include_router(st.router)
        TOPICS.append(st)
        logger.info("ServerTelemetry initialized successfully!")
    except Exception as e:
        logger.error(f"Error when initializing server telemetry endpoint! {e}")

    for msg in CONFIG["topics"]:
        try:
            th = NodeHandler(msg)
            app.include_router(th.router)
            TOPICS.append(th)
            logger.info(f"{msg['msg_type']} NodeHandler initialized successfully!")
        except Exception as e:
            logger.warning(f"Couldn't create/start NodeHandler for {msg['msg_type']}:", e)
        sleep(0.1)
    yield
    shutdown()

app = FastAPI(lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

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
    parser.add_argument('--host', type=str, default='localhost', help='Host address of the server')
    parser.add_argument('--port', type=int, default=8000, help='Port number of the server')
    args = parser.parse_args()

    try:
        uvicorn.run(app, host=args.host, port=args.port)
    except KeyboardInterrupt:
        logger.info('KeyboardInterrupt received, shutting down...')
        shutdown()
