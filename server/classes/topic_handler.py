from fastapi import APIRouter, Request, HTTPException
from classes.topic_receiver import TopicReceiver
import threading
from rclpy import spin
from sse_starlette.sse import EventSourceResponse
import asyncio

from database.influx_client import (
    InfluxClient,
    InfluxNotAvailableException,
    BucketNotFoundException,
    BadQueryException,
)

class TopicHandler:

    def __init__(self, msg_config):
        
        self.load_config(msg_config)
        self.receiver = TopicReceiver(
            self.msg_type, self.topic_name, self.interval, self.msg_fields)
        
        self.router = APIRouter()
        self.router.add_api_route(
            f"/{self.topic_name}", self.return_msg, methods=["GET"])
        self.router.add_api_route(
            f"/{self.topic_name}/query", self.query, methods=["GET"])
        
        self.thread = threading.Thread(target=spin, args=(self.receiver,))
        self.thread.start()
        

    async def return_msg(self, r: Request):
        
        # TODO: check for new messages
        def new_messages():
            return True

        async def event_generator():
            while True:
                if await r.is_disconnected():
                    break
                
                if new_messages():
                    msg = self.receiver.get_msg()
                    self.tmp_msg = msg
                    # yield f"data: {msg}\n\n"
                    yield msg
            
        await asyncio.sleep(self.interval/2000)
        return EventSourceResponse(event_generator(), media_type="text/event-stream")
    
    async def query(self, r: Request, time_range: int = 10):
        ic = InfluxClient()
        try:
            records = []
            # records = await ic.query_(msg_type: str, val_name: str, time_range=time_range)
        except (
            InfluxNotAvailableException,
            BucketNotFoundException,
            BadQueryException,
        ) as e:
            raise HTTPException(
                status_code=e.STATUS_CODE,
                detail=e.DESCRIPTION,
            )
        # return ListBucketResponse(records=records)

    def load_config(self, msg_config):
        self.msg_fields = msg_config["msg_fields"]
        self.msg_type = msg_config["msg_type"]
        self.topic_name = msg_config["topic_name"]
        self.interval = msg_config["interval"]
