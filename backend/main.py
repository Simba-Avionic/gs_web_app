import rclpy
import json
import signal
import sys
import os
import threading
import subprocess
import requests
from requests.auth import HTTPDigestAuth
from pydantic import BaseModel

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

import mimetypes
mimetypes.add_type("application/vnd.apple.mpegurl", ".m3u8")
mimetypes.add_type("video/mp2t", ".ts")

from fastapi.staticfiles import StaticFiles


# ------------------ CAMERA CONFIG ------------------
CAMERA_RTSP_URL = "rtsp://admin:simba123@192.168.1.200:554/Streaming/Channels/101"
CAMERA_HLS_DIR = "static/camera"
CAMERA_HLS_FILE = os.path.join(CAMERA_HLS_DIR, "stream.m3u8")

# Ensure HLS output directory exists
os.makedirs(CAMERA_HLS_DIR, exist_ok=True)


ffmpeg_process = None
# ----------------------------------------------------
class PTZCommand(BaseModel):
    pan: float  # -1.0 to 1.0 (left to right)
    tilt: float # -1.0 to 1.0 (down to up)
    zoom: float # -1.0 to 1.0 (zoom out to in)
    speed: float # 0 to 1

def send_ptz_command(cmd: PTZCommand):
    url = f"http://192.168.1.200/PTZCtrl/channels/1/continuous"
    headers = {"Content-Type": "application/xml"}

    xml_body = f"""<?xml version="1.0" encoding="UTF-8"?>
    <PTZData>
      <pan>{cmd.pan}</pan>
      <tilt>{cmd.tilt}</tilt>
      <zoom>{cmd.zoom}</zoom>
      <speed>{cmd.speed}</speed>
    </PTZData>
    """

    response = requests.put(url, headers=headers, data=xml_body, auth=HTTPDigestAuth("admin", "simba123"))
    if response.status_code != 200:
        raise HTTPException(status_code=response.status_code, detail=f"Failed to send PTZ command: {response.text}")

def start_camera_stream():
    """Start FFmpeg process to convert RTSP to HLS for browser playback."""
    global ffmpeg_process
    logger.info(CAMERA_HLS_FILE)

    if ffmpeg_process and ffmpeg_process.poll() is None:
        logger.info("Camera streaming already running.")
        return

    logger.info(f"Starting FFmpeg for camera stream from {CAMERA_RTSP_URL}...")
    ffmpeg_process = subprocess.Popen([
    "ffmpeg",
    "-rtsp_transport", "tcp",
    "-i", CAMERA_RTSP_URL,
    "-c", "copy",
    "-f", "hls",
    "-hls_time", "2",
    "-hls_list_size", "3",
    "-hls_flags", "delete_segments+append_list",
    CAMERA_HLS_FILE,
], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def stop_camera_stream():
    """Stop FFmpeg process."""
    global ffmpeg_process
    if ffmpeg_process:
        ffmpeg_process.terminate()
        ffmpeg_process.wait()
        ffmpeg_process = None
        logger.info("Camera stream stopped.")


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

    # try:
    #     mavlink_client = MavlinkClient()
    # except Exception as e:
    #     logger.error(f"Error when initializing Mavlink client: {e}")

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

    start_camera_stream()

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

app.mount("/camera", StaticFiles(directory=CAMERA_HLS_DIR), name="camera")


@app.post("/ptz/move")
def ptz_move(cmd: PTZCommand):
    send_ptz_command(cmd)
    return {"message": "PTZ command sent"}

# --------------- CAMERA ROUTE ----------------
@app.get("/camera/stream.m3u8")
def get_camera_stream():
    if not os.path.isfile(CAMERA_HLS_FILE):
        raise HTTPException(status_code=404, detail="Camera stream not ready")
    return FileResponse(CAMERA_HLS_FILE, media_type="application/vnd.apple.mpegurl")
# ----------------------------------------------

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
