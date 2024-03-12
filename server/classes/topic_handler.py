from fastapi import FastAPI, APIRouter, Response, Request
from classes.topic_receiver import TopicReceiver
import threading
from rclpy import init, shutdown, spin
from sse_starlette.sse import EventSourceResponse
import asyncio

class TopicHandler:

    def __init__(self, msg_config):
        
        self.load_config(msg_config)
        self.receiver = TopicReceiver(
            self.msg_type, self.topic_name, self.interval, self.msg_fields)
        
        self.router = APIRouter()
        self.router.add_api_route(
            f"/{self.topic_name}", self.return_msg, methods=["GET"])
        
        self.thread = threading.Thread(target=spin, args=(self.receiver,))
        self.thread.start()

    async def return_msg(self, request: Request):
        # TODO: check for new messages
        def new_messages():
            return True

        async def event_generator():
            while True:
                if await request.is_disconnected():
                    break
                
                if new_messages():
                    msg = self.receiver.get_msg()
                    yield f"data: {msg}\n\n"
            
        await asyncio.sleep(self.interval/2000)
        return EventSourceResponse(event_generator(), media_type="text/event-stream")
    
    def load_config(self, msg_config):
        self.msg_fields = msg_config["msg_fields"]
        self.msg_type = msg_config["msg_type"]
        self.topic_name = msg_config["topic_name"]
        self.interval = msg_config["interval"]
