from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from rclpy import init, shutdown, spin
from rclpy.node import Node
from std_msgs.msg import Int32
import threading
from contextlib import asynccontextmanager

from routes.read import read_router
from routes.write import write_router

subscriber = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    global subscriber
    global spin_thread
    init()
    subscriber = NumberSubscriber()
    spin_thread = threading.Thread(target=spin, args=(subscriber,))
    spin_thread.start()
    yield
    if subscriber is not None:
        subscriber.destroy_node()
        spin_thread.join()
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

class NumberSubscriber(Node):
    def __init__(self):
        super().__init__('number_subscriber')
        self.subscription = self.create_subscription(
            Int32, 'latest_number', self.number_callback, 10)
        self.latest_number = None

    def number_callback(self, msg):
        self.latest_number = msg.data

@app.get("/latest-number")
async def get_latest_number():
    if subscriber is None:
        raise HTTPException(status_code=500, detail="ROS not initialized")
    return {"latest_number": subscriber.latest_number}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="localhost", port=8000)