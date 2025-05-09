from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from influxdb_client.rest import ApiException
from typing import List, Any
from loguru import logger
from urllib3.exceptions import NewConnectionError
from database.schemas import BadQueryException, BucketNotFoundException, InfluxNotAvailableException
from datetime import datetime
from dotenv import dotenv_values, find_dotenv
import json

class InfluxClient:

    def __init__(self, msg_type: str, topic_name: str, msg_fields: dict) -> None:
        env_values = dotenv_values(find_dotenv())
        self.bucket = env_values.get('BUCKET_NAME')
        self.org = env_values.get('ORGANISATION')
        self.token = env_values.get('INFLUXDB_TOKEN')
        self.url = f"{env_values.get('IP_ADDRESS')}:{env_values.get('INFLUXDB_PORT')}"
        self._client = InfluxDBClient(url=self.url, token=self.token, org=self.org)

        self.msg_type = msg_type
        self.topic_name = topic_name
        self.msg_fields = msg_fields

    def db_alive(self) -> bool:
        return self._client.ping()

    def insert_data(self, msg_data) -> None:
        p = Point(self.msg_type)

        def add_fields(prefix, value):
            if isinstance(value, dict):
                for key, sub_value in value.items():
                    add_fields(f"{prefix}_{key}" if prefix else key, sub_value)
            else:
                if prefix == "header":
                    stamp = value['stamp']['sec'] * 1000 + value['stamp']['nanosec'] / 1000000
                    p.time(int(stamp), WritePrecision.MS)
                elif prefix == "header_stamp_sec" or prefix == "header_stamp_nanosec":
                    return
                elif prefix == "header_frame_id":
                    p.tag("frame_id", value)
                else:
                    p.field(prefix, value)
        
        add_fields("", msg_data)
        self._insert(p)

    def _insert(self, p: Point) -> Any:
        write_api = self._client.write_api(write_options=SYNCHRONOUS)
        try:
            res = write_api.write(bucket=self.bucket, org=self.org, record=p)
        except NewConnectionError:
            raise InfluxNotAvailableException()
        except ApiException as e:
            if e.status and e.status == 400:
                raise BadQueryException()
            if e.status and e.status == 404:
                raise BucketNotFoundException()
            raise InfluxNotAvailableException()
        # logger.debug(f"{p}")
        return res
    
    """
    async def _query(self, query: str = "") -> List[InfluxWaveRecord]:
        logger.debug(f"Running {query=}")
        query_api = self._client.query_api()
        try:
            result = query_api.query(query=query)
        except NewConnectionError:
            raise InfluxNotAvailableException()
        except ApiException as e:
            if e.status and e.status == 400:
                raise BadQueryException()
            if e.status and e.status == 404:
                raise BucketNotFoundException()
            raise InfluxNotAvailableException()
        res = []
        for table in result:
            for record in table.records:
                r = InfluxWaveRecord(
                    location=record.values.get("location"), height=record.get_value()
                )
                res.append(r)
        logger.debug(f"Query returned {len(res)} records")
        return res
    async def read_wave_height(
        self, location: str = "", min_height: float = -1.0, time_range: int = 10
    ) -> List[InfluxWaveRecord]:
        query = f'from(bucket:"{self.bucket}")\
            |> range(start: -{time_range}m) \
            |> filter(fn:(r) => r._measurement == "{InfluxClient.MEASUREMENT_NAME}")'
        if location:
            location = location.lower()
            query += f'|> filter(fn:(r) => r.location == "{location}")'
        if min_height > 0:
            query += f'|> filter(fn:(r) => r._field >= "{min_height}")'
        return await self._query(query)
    """