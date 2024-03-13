from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
from influxdb_client.rest import ApiException
from typing import List, Any
from loguru import logger
from urllib3.exceptions import NewConnectionError
from database.schemas import BadQueryException, BucketNotFoundException, InfluxNotAvailableException

from dotenv import dotenv_values
ENV_PATH = "./database/.env"


class InfluxClient:

    def __init__(self) -> None:
        env_values = dotenv_values(ENV_PATH)
        self.bucket = env_values.get('BUCKET_NAME')
        self.org = env_values.get('ORGANISATION')
        self.token = env_values.get('INFLUXDB_TOKEN')
        self.url = f"http://localhost:{env_values.get('INFLUXDB_PORT')}"
        self._client = InfluxDBClient(url=self.url, token=self.token, org=self.org)

    """
    async def record_wave_height(self, location: str, height: float) -> None:
        location = location.lower()
        p = (
            Point(InfluxClient.MEASUREMENT_NAME)
            .tag("location", location)
            .field("height", height)
        )
        await self._insert(p)

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

    async def _insert(self, p: Point) -> Any:
        print(self.bucket, self.org)
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
        logger.info(f"{res=}")
        return res

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
    """