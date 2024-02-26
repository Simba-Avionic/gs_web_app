from fastapi import APIRouter, Request, HTTPException
from loguru import logger

from schemas import ListBucketResponse
from influx import (
    InfluxWaveClient,
    InfluxNotAvailableException,
    BucketNotFoundException,
    BadQueryException,
)

from dotenv import dotenv_values
env_values = dotenv_values("../database/.env")

"""
Let's move on to the read router. This router
will include the following endpoints: `/read/<bucket>/query` and
`/read/<bucket>/list`. 

The query endpoint will take two optional query parameters from the`
caller: location and min_height. It will then instantiate our client
using the server's settings (discussed later). It then calls the 
`read_wave_height` client method and then return all of the matching
data points to the caller.

The list endpoint does almost the same thing except, it doesn't have
any query parameters because we really just want all of the data points
from the bucket. So, this method instantiates the client and then calls
the `list_wave_heights` method and returns the data points to the caller.
"""
read_router = APIRouter(prefix="/read")


@read_router.get(
    "/{bucket}/query",
    summary="Queries a bucket's contents.",
    responses={
        200: {"description": "Successfully Queried Bucket."},
        400: {"description": "Bad Filter Requested."},
        404: {"description": "Bucket not found."},
        503: {"description": "InfluxDB Not Available"},
    },
)
async def query_bucket(
    r: Request, bucket: str, location: str = "", min_height: float = -1.0
) -> ListBucketResponse:
    logger.debug(f"Querying {bucket=} with {location=} and {min_height}")
    ic = InfluxWaveClient(
        env_values.get('BUCKET_NAME'),
        env_values.get('INFLUXDB_TOKEN'), 
        env_values.get('ORGANISATION'), 
        f"http://localhost:{env_values.get('INFLUXDB_PORT')}"
    )
    try:
        records = await ic.read_wave_height(location=location, min_height=min_height)
    except (
        InfluxNotAvailableException,
        BucketNotFoundException,
        BadQueryException,
    ) as e:
        raise HTTPException(
            status_code=e.STATUS_CODE,
            detail=e.DESCRIPTION,
        )
    logger.debug(f"Records fetched {records=}")
    return ListBucketResponse(bucket=bucket, records=records)


@read_router.get(
    "/{bucket}/list",
    summary="List's a bucket's contents.",
    responses={
        200: {"description": "Successfully Listed Bucket."},
        404: {"description": "Bucket not found."},
        503: {"description": "InfluxDB Not Available"},
    },
)
async def list_bucket(r: Request, bucket: str) -> ListBucketResponse:
    logger.debug(f"Listing {bucket=}")
    ic = InfluxWaveClient(
        env_values.get('BUCKET_NAME'),
        env_values.get('INFLUXDB_TOKEN'), 
        env_values.get('ORGANISATION'), 
        f"http://localhost:{env_values.get('INFLUXDB_PORT')}"
    )
    try:
        records = await ic.list_wave_heights()
    except (
        InfluxNotAvailableException,
        BucketNotFoundException,
        BadQueryException,
    ) as e:
        raise HTTPException(
            status_code=e.STATUS_CODE,
            detail=e.DESCRIPTION,
        )
    return ListBucketResponse(bucket=bucket, records=records)