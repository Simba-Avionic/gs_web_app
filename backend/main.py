import rclpy
import json
import asyncio
import sys
import os
import threading
from loguru import logger
from fastapi import FastAPI
from contextlib import asynccontextmanager
from fastapi.responses import JSONResponse, FileResponse
from fastapi.middleware.cors import CORSMiddleware
from rclpy.executors import MultiThreadedExecutor
from dotenv import dotenv_values, find_dotenv
from fastapi.staticfiles import StaticFiles

from src.node_handler import NodeHandler
from src.server_telemetry import ServerTelemetry
from src.camera_handler import router as camera_router, start_all_camera_streams, BASE_HLS_DIR

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


@asynccontextmanager
async def lifespan(app: FastAPI):
    rclpy.init()
    load_main_config(CONFIG_JSON_PATH)
    executor = MultiThreadedExecutor()

    global TOPICS
    TOPICS = []

    # FastAPI event loop where websocket broadcasts occur
    app_loop = asyncio.get_event_loop()
    import queue
    write_queue = queue.Queue()

    # Start a single writer thread that consumes the write_queue
    from database.influx_client import InfluxClient
    db_client = InfluxClient()

    def influx_writer_task(q, shared_ic):
        import time
        from queue import Empty
        logger.info("Influx writer thread started.")
        BATCH_MAX = 100
        BATCH_TIMEOUT = 2

        while True:
            # first blocking get
            try:
                item = q.get()
            except Exception as e:
                logger.warning(f"Writer queue get error: {e}")
                time.sleep(0.05)
                continue

            total_gets = 1  # we have consumed one get()
            if item is None:
                q.task_done()
                break

            batch = [item]
            start = time.time()
            # try to drain more items
            while len(batch) < BATCH_MAX:
                remaining = BATCH_TIMEOUT - (time.time() - start)
                if remaining <= 0:
                    break
                try:
                    more = q.get(timeout=remaining)
                    batch.append(more)
                    total_gets += 1
                except Empty:
                    break
                except Exception as e:
                    logger.debug(f"Writer queue get (non-empty) error: {e}")
                    break

            # Detect sentinel(s) and strip them
            stop_after = any(x is None for x in batch)
            batch = [x for x in batch if x is not None]

            # Build Points using shared client helper and write them in one call
            points = []
            for item in batch:
                # expecting items shaped as (measurement, msg)
                try:
                    measurement, msg = item
                except Exception:
                    logger.warning(f"Unexpected queue item shape: {item}")
                    continue
                try:
                    p = shared_ic._point_from_msg(msg, measurement)
                    points.append(p)
                except Exception as e:
                    logger.warning(f"Failed to build point: {e}")

            if points:
                try:
                    shared_ic.insert_points(points)
                    # logger.debug(f"Influx batch write with {len(points)} points")
                except Exception as e:
                    logger.warning(f"Influx batch write failed: {e}")

            # mark task_done for each q.get() call we did
            for _ in range(total_gets):
                try:
                    q.task_done()
                except Exception:
                    pass

            if stop_after:
                break

        logger.info("Influx writer thread stopping.")

    writer_thread = threading.Thread(target=influx_writer_task, args=(write_queue, db_client), daemon=True)
    writer_thread.start()

    try:
        st = ServerTelemetry(db_client)
        app.include_router(st.router)
        logger.info("ServerTelemetry initialized successfully!")
    except Exception as e:
        logger.error(f"Error when initializing server telemetry endpoint: {e}")

    for msg in CONFIG["topics"]:
        try:
            nh = NodeHandler(msg, app_loop=app_loop, write_queue=write_queue, influx_client=db_client)
            executor.add_node(nh)
            app.include_router(nh.router)
            TOPICS.append(nh)
            logger.info(f"{msg['msg_type']} NodeHandler initialized successfully!")
        except Exception as e:
            logger.warning(f"Couldn't create/start NodeHandler for {msg['msg_type']}: {e}")

    start_all_camera_streams()

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    yield

    # Shutdown sequence
    try:
        write_queue.put(None)
        writer_thread.join(timeout=3)
    except Exception as e:
        logger.warning(f"Error stopping writer thread: {e}")

    executor.shutdown()
    executor_thread.join(timeout=3)

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

app.include_router(camera_router)
app.mount("/camera", StaticFiles(directory=BASE_HLS_DIR), name="camera")


@app.get("/tiles/{layer}/{z}/{x}/{y}.png")
def get_tile(layer: str, z: int, x: int, y: int):
    tile_path = os.path.join(TILES_DIRECTORY, layer, str(z), str(x), f"{y}.png")
    if not os.path.isfile(tile_path):
        from fastapi import HTTPException
        raise HTTPException(status_code=404, detail="Tile not found")
    return FileResponse(tile_path, media_type="image/png")


@app.get("/config")
async def get_config():
    return JSONResponse(content=CONFIG)


if __name__ == "__main__":
    import uvicorn, argparse

    parser = argparse.ArgumentParser(description="Start the Uvicorn server.")
    parser.add_argument('--host', type=str, default=env_values.get("IP_ADDRESS"), help='Host address of the server')
    parser.add_argument('--port', type=int, default=8000, help='Port number of the server')
    args = parser.parse_args()

    try:
        uvicorn.run(app, host=args.host, port=args.port)
    except Exception as e:
        logger.error(f"Error running Uvicorn: {e}")
