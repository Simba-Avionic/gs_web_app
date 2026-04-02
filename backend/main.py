import rclpy
import json
import asyncio
import sys
import os
import threading
import queue
from loguru import logger
from fastapi import FastAPI, HTTPException
from contextlib import asynccontextmanager
from fastapi.responses import JSONResponse, FileResponse
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from rclpy.executors import MultiThreadedExecutor
from dotenv import find_dotenv, load_dotenv
from fastapi.staticfiles import StaticFiles

from src.node_handler import NodeHandler
from src.server_telemetry import ServerTelemetry
from src.camera_handler import router as camera_router, start_all_camera_streams, BASE_HLS_DIR
from database.influx_client import InfluxClient

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "mavlink"))
from xml_to_json import extract_enums

if not rclpy.ok():
    rclpy.init()

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from shared.paths import CONFIG_JSON_PATH, TILES_DIRECTORY, SIMBA_XML_PATH

load_dotenv(find_dotenv())

with open(CONFIG_JSON_PATH, "r") as f:
    CONFIG = json.load(f)

try:
    enums = extract_enums(SIMBA_XML_PATH)
    enums_output_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "enums.json")
    with open(enums_output_path, "w") as f:
        json.dump(enums, f, indent=2)
    logger.info(f"Generated enums.json at {enums_output_path}")
except Exception as e:
    logger.error(f"Failed to generate enums.json: {e}")

db_client = InfluxClient()
write_queue = queue.Queue()
executor = MultiThreadedExecutor()
TOPICS = []

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DIST_DIR = os.path.join(BASE_DIR, "frontend", "dist")

app = FastAPI()

for msg in CONFIG["topics"]:
    try:
        nh = NodeHandler(msg, app_loop=None, write_queue=write_queue, influx_client=db_client)
        app.include_router(nh.router)
        TOPICS.append(nh)
        logger.info(f"Route for {msg['topic_name']} registered globally.")
    except Exception as e:
        logger.error(f"Failed to create NodeHandler for {msg['msg_type']}: {e}")

@asynccontextmanager
async def lifespan(app: FastAPI):
    loop = asyncio.get_running_loop()
    logger.info("Asyncio loop captured!")

    for nh in TOPICS:
        nh.app_loop = loop
        executor.add_node(nh)
    
    try:
        st = ServerTelemetry(db_client)
        app.include_router(st.router)
        logger.info("ServerTelemetry initialized with active loop.")
    except Exception as e:
        logger.error(f"Telemetry error: {e}")

    def influx_writer():
        while True:
            item = write_queue.get()
            if item is None: break
            try:
                m, msg = item
                p = db_client._point_from_msg(msg, m)
                db_client.insert_points([p])
            except: pass
            finally: write_queue.task_done()

    threading.Thread(target=influx_writer, daemon=True).start()
    threading.Thread(target=start_all_camera_streams, daemon=True).start()
    
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    
    yield
    executor.shutdown()
    rclpy.shutdown()

app.router.lifespan_context = lifespan

app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_credentials=False, allow_methods=["*"], allow_headers=["*"])
app.add_middleware(TrustedHostMiddleware, allowed_hosts=["*"])

app.include_router(camera_router)
app.mount("/camera", StaticFiles(directory=BASE_HLS_DIR), name="camera")

@app.get("/config")
async def get_config(): return JSONResponse(content=CONFIG)

@app.get("/enums")
async def get_enums():
    """Serve enum mappings from the XML"""
    try:
        enums_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "enums.json")
        with open(enums_path, "r") as f:
            enums = json.load(f)
        return enums
    except FileNotFoundError:
        return {"error": "enums.json not found. Ensure server has started successfully."}

@app.get("/tiles/{layer}/{z}/{x}/{y}.png")
def get_tile(layer: str, z: int, x: int, y: int):
    tile_path = os.path.join(TILES_DIRECTORY, layer, str(z), str(x), f"{y}.png")
    if os.path.isfile(tile_path):
        return FileResponse(tile_path, media_type="image/png")
    raise HTTPException(status_code=404)

if os.path.exists(DIST_DIR):
    app.mount("/assets", StaticFiles(directory=os.path.join(DIST_DIR, "assets")), name="assets")

    @app.get("/{rest_of_path:path}")
    async def serve_spa(rest_of_path: str):
        full_path = os.path.join(DIST_DIR, rest_of_path)
        if rest_of_path and os.path.isfile(full_path):
            return FileResponse(full_path)
        return FileResponse(os.path.join(DIST_DIR, "index.html"))

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host=os.getenv("IP_ADDRESS", "0.0.0.0"), port=2137)