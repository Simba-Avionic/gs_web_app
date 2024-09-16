import rclpy
import json
import signal
import sys

from time import sleep
from fastapi import FastAPI
from contextlib import asynccontextmanager
from fastapi.responses import JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from src.node_handler import NodeHandler

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
        print(f"Error during ROS2 shutdown: {e}")

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

    for msg in CONFIG["topics"]:
        try:
            th = NodeHandler(msg)
            app.include_router(th.router)
            TOPICS.append(th)
            print(f"{msg['msg_type']} NodeHandler initialized successfully!")
        except Exception as e:
            print(f"Couldn't create/start NodeHandler for {msg['msg_type']}:", e)
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
    print('SIGINT received, shutting down...')
    shutdown()
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    import uvicorn
    try:
        uvicorn.run(app, host="localhost", port=8000)
    except KeyboardInterrupt:
        print('KeyboardInterrupt received, shutting down...')
        shutdown()
