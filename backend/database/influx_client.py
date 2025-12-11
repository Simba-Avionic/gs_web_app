from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from influxdb_client.rest import ApiException
from typing import List, Any, Dict
from influxdb_client.client.query_api import QueryApi
from urllib3.exceptions import NewConnectionError
from database.schemas import BadQueryException, BucketNotFoundException, InfluxNotAvailableException
from dotenv import dotenv_values, find_dotenv

class InfluxClient:

    def __init__(self) -> None:
        env_values = dotenv_values(find_dotenv())
        self.bucket = env_values.get('BUCKET_NAME')
        self.org = env_values.get('ORGANISATION')
        self.token = env_values.get('INFLUXDB_TOKEN')
        self.url = f"{env_values.get('IP_ADDRESS')}:{env_values.get('INFLUXDB_PORT')}"
        self._client = InfluxDBClient(url=self.url, token=self.token, org=self.org)
        self.write_api = self._client.write_api(write_options=SYNCHRONOUS)

    def db_alive(self) -> bool:
        return self._client.ping()
    
    def _point_from_msg(self, msg_data, measurement: str | None = None) -> Point:
        """
        Convert a message dict into an InfluxDB Point.
        If measurement is provided, use it; otherwise fall back to a default name.
        """
        meas = measurement or "unknown_measurement"
        p = Point(meas)

        def add_fields(prefix, value):
            if isinstance(value, dict):
                for key, sub_value in value.items():
                    add_fields(f"{prefix}/{key}" if prefix else key, sub_value)
            else:
                if prefix == "header":
                    # header is an object; expect dict with stamp
                    try:
                        stamp = value['stamp']['sec'] * 1000 + value['stamp']['nanosec'] / 1000000
                        p.time(int(stamp), WritePrecision.MS)
                    except Exception:
                        pass
                elif prefix == "header_stamp_sec" or prefix == "header_stamp_nanosec":
                    return
                elif prefix == "header_frame_id":
                    p.tag("frame_id", value)
                else:
                    # flatten field name and use as field
                    p.field(prefix, value)

        add_fields("", msg_data)
        return p

    def insert_batch(self, messages: List[dict], measurement: str | None = None) -> Any:
        """
        Insert a batch of messages (list of message dicts) in a single write operation.
        If measurement is provided, use it for all messages in this batch.
        """
        pts = []
        for msg in messages:
            try:
                pts.append(self._point_from_msg(msg, measurement))
            except Exception:
                continue

        if not pts:
            return None

        try:
            # write accepts a list of Points
            res = self.write_api.write(bucket=self.bucket, org=self.org, record=pts)
        except NewConnectionError:
            raise InfluxNotAvailableException()
        except ApiException as e:
            if e.status and e.status == 400:
                raise BadQueryException()
            if e.status and e.status == 404:
                raise BucketNotFoundException()
            raise InfluxNotAvailableException()
        return res

    def insert_points(self, points: List[Point]) -> Any:
        """
        Write a pre-built list of Points in a single API call.
        """
        if not points:
            return None
        try:
            res = self.write_api.write(bucket=self.bucket, org=self.org, record=points)
        except NewConnectionError:
            raise InfluxNotAvailableException()
        except ApiException as e:
            if e.status and e.status == 400:
                raise BadQueryException()
            if e.status and e.status == 404:
                raise BucketNotFoundException()
            raise InfluxNotAvailableException()
        return res

    def insert_data(self, msg_data) -> None:
        """
        Insert a single message as a Point (keeps backward compatibility).
        """
        p = self._point_from_msg(msg_data)
        self._insert(p)

    def _insert(self, p: Point) -> Any:
        try:
            res = self.write_api.write(bucket=self.bucket, org=self.org, record=p)
        except NewConnectionError:
            raise InfluxNotAvailableException()
        except ApiException as e:
            if e.status and e.status == 400:
                raise BadQueryException()
            if e.status and e.status == 404:
                raise BucketNotFoundException()
            raise InfluxNotAvailableException()
        return res
    
    def query_data(self, flux_query: str) -> List[Dict[str, Any]]:
        """
        Query data from InfluxDB using a Flux query string.
        
        Args:
            flux_query (str): Flux query string
        
        Returns:
            List[Dict[str, Any]]: List of query result records as dictionaries
        
        Raises:
            InfluxNotAvailableException, BadQueryException, BucketNotFoundException
        """
        query_api: QueryApi = self._client.query_api()
        
        try:
            tables = query_api.query(flux_query, org=self.org)
        except NewConnectionError:
            raise InfluxNotAvailableException()
        except ApiException as e:
            if e.status == 400:
                raise BadQueryException()
            if e.status == 404:
                raise BucketNotFoundException()
            raise InfluxNotAvailableException()

        results = []
        for table in tables:
            for record in table.records:
                rec_dict = {
                    **record.values
                }
                results.append(rec_dict)
        return results