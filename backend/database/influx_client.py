from influxdb import InfluxDBClient
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
        self.db = 'simba'  # InfluxDB 1.x uses "database"
        self.host = env_values.get('IP_ADDRESS')
        self.port = int(env_values.get('INFLUXDB_PORT', 8086))
        self.username = env_values.get('INFLUXDB_USER')
        self.password = env_values.get('INFLUXDB_PASSWORD')
        self._client = InfluxDBClient(
            host=self.host,
            port=self.port,
            username=self.username,
            password=self.password,
            database=self.db
        )

        self.msg_type = msg_type
        self.topic_name = topic_name
        self.msg_fields = msg_fields

    def db_alive(self) -> bool:
        try:
            return self._client.ping()
        except Exception:
            return False

    def insert_data(self, msg_data) -> None:
        point = {
            "measurement": self.msg_type,
            "tags": {},
            "fields": {},
            "time": None
        }

        def add_fields(prefix, value):
            if isinstance(value, dict):
                for key, sub_value in value.items():
                    add_fields(f"{prefix}_{key}" if prefix else key, sub_value)
            else:
                if prefix == "header":
                    # Expecting value to be a dict with 'stamp'
                    stamp = value['stamp']['sec'] * 1000 + value['stamp']['nanosec'] / 1000000
                    # InfluxDB expects RFC3339 or ISO8601, so convert ms to ISO format
                    dt = datetime.utcfromtimestamp(stamp / 1000.0)
                    point["time"] = dt.isoformat() + "Z"
                elif prefix == "header_stamp_sec" or prefix == "header_stamp_nanosec":
                    return
                elif prefix == "header_frame_id":
                    point["tags"]["frame_id"] = value
                else:
                    # InfluxDB fields must be int, float, bool, or str
                    point["fields"][prefix] = value

        add_fields("", msg_data)

        # Remove None time if not set
        if point["time"] is None:
            point.pop("time")

        # TODO: Remove it / Refactor
        # Only write if there are fields
        if not point["fields"]:
            logger.warning(f"Skipping InfluxDB write: no fields for measurement {self.msg_type}")
            return

        try:
            res = self._client.write_points([point])
            if not res:
                raise InfluxNotAvailableException("Failed to write point to InfluxDB")
        except NewConnectionError:
            raise InfluxNotAvailableException()
        except Exception as e:
            logger.error(f"InfluxDB write error: {e}")
            raise InfluxNotAvailableException()

    # You can implement query methods similarly using self._client.query()