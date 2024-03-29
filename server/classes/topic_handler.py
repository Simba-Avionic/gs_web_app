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
        
        self.ic = InfluxClient()
        
        self.thread = threading.Thread(target=spin, args=(self.receiver,))
        self.thread.start()

        self.last_timestamp = None
        self.curr_msg = None
        

    async def return_msg(self, r: Request):
        
        # TODO: check for new messages
        def new_messages():
            self.curr_msg = self.receiver.get_msg()
            if self.curr_msg == None:
                return False
            # if self.curr_msg.header.stamp == self.last_timestamp:
            #     return False
            return True

        async def event_generator():
            while True:
                if await r.is_disconnected():
                    break
                if new_messages():
                    yield self.curr_msg.data
                else:
                    yield None            
                await asyncio.sleep(self.interval/1000)
        
        return EventSourceResponse(event_generator(), media_type="text/event-stream")
    
    async def insert(self, msg):
        self.ic.insert_data(msg)
    
    async def query(self, r: Request, time_range: int = 5):
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
