from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from rclpy import init, shutdown, spin
from contextlib import asynccontextmanager
import json
from classes.topic_handler import TopicHandler

from routes.read import read_router
from routes.write import write_router

CONFIG = None
CONFIG_PATH = "test_config.json"
TOPICS = []

def load_main_config(path_to_conf):
    global CONFIG
    f = open(path_to_conf)
    CONFIG = json.load(f)
    f.close()

@asynccontextmanager
async def lifespan(app: FastAPI):
    init()
    load_main_config(CONFIG_PATH)

    for msg in CONFIG["topics"]:
        # print(msg)
        try:
            th = TopicHandler(msg)
            app.include_router(th.router)
            TOPICS.append(th)
        except Exception as e:
            print(f"Couldn't create/start TopicHandler for {msg['msg_type']}", e)
    yield
    shutdown()

app = FastAPI(lifespan=lifespan)
app.include_router(read_router)
app.include_router(write_router)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["GET"],
    allow_headers=["*"],
)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="localhost", port=8000)