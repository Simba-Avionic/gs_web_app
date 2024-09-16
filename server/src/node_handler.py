import time
import asyncio
import threading

from loguru import logger
from rclpy import executors
from fastapi import WebSocket
from fastapi import APIRouter, Request, HTTPException
from src.topic_receiver import TopicReceiver

from database.influx_client import (
    InfluxClient,
    InfluxNotAvailableException,
    BucketNotFoundException,
    BadQueryException,
)

class NodeHandler:

    def __init__(self, msg_config):
        
        self.load_config(msg_config)
        self.receiver = TopicReceiver(
            self.msg_type, self.topic_name, self.interval, self.msg_fields)
        
        self.router = APIRouter()
        self.router.add_api_websocket_route(f"/{self.topic_name}", self.return_msg)
        self.router.add_api_websocket_route(f"/{self.topic_name}/query", self.query)
       
        self.executor = executors.MultiThreadedExecutor()
        self.executor.add_node(self.receiver)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        # DB thread is for later data usage and rendundancy if REST API would fail
        self.ic = InfluxClient(self.msg_type, self.topic_name, self.msg_fields)
        
        if self.ic:
            self.db_insert = threading.Thread(target=self.db_thread)
            self.db_insert.start()

    async def return_msg(self, ws: WebSocket):
        last_msg = None
        await ws.accept()
        try:
            while True:
                await asyncio.sleep(self.interval / 1000)
                data = self.receiver.get_msg()
                if data:
                    last_msg = data
                    await ws.send_json(data)
                elif last_msg and (time.time() - int(last_msg['header']['stamp']['sec'])) > (self.interval / 100):
                    await ws.send_json(None)
                    
        except Exception as e:
            logger.error(f"WebSocket connection closed: {e}")
        finally:
            await ws.close()

    # TODO: query data for plots
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

    def db_thread(self):
        while True:
            data = self.receiver.get_msg_db()
            if data:
                self.ic.insert_data(data)
            time.sleep(self.interval/1000)

    def load_config(self, msg_config):
        self.msg_fields = msg_config["msg_fields"]
        self.msg_type = msg_config["msg_type"]
        self.topic_name = msg_config["topic_name"]
        self.interval = msg_config["interval"]

    def stop(self):
        self.executor.remove_node(self.receiver)
        self.receiver.destroy_node()
        self.executor.shutdown()
        self.executor_thread.join(timeout=5)
        logger.info(f"NodeHandler stopped for {self.receiver}")