from fastapi import FastAPI
from fastapi.responses import JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from rclpy import init, shutdown
from contextlib import asynccontextmanager
import json
from classes.topic_handler import TopicHandler

CONFIG = None
CONFIG_PATH = "config.json"
TOPICS = []

def load_main_config(path_to_conf):
    global CONFIG
    with open(path_to_conf, "r") as f:
        CONFIG = json.load(f)

@asynccontextmanager
async def lifespan(app: FastAPI):
    init()
    load_main_config(CONFIG_PATH)

    for msg in CONFIG["topics"]:
        try:
            th = TopicHandler(msg)
            app.include_router(th.router)
            TOPICS.append(th)
        except Exception as e:
            print(f"Couldn't create/start TopicHandler for {msg['msg_type']}", e)
    yield
    shutdown()
    for topic in TOPICS:
        topic.thread.join()
 
app = FastAPI(lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["GET"],
    allow_headers=["*"],
)

@app.get("/config")
async def get_config():
    return JSONResponse(content=CONFIG)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="localhost", port=8000)