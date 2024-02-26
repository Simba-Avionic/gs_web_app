from fastapi import APIRouter, HTTPException
from loguru import logger
from schemas import InsertWaveHeightRequest, InsertWaveHeightResponse
from influx import (
    InfluxWaveClient,
    InfluxNotAvailableException,
    BucketNotFoundException,
    BadQueryException,
)

from dotenv import dotenv_values
env_values = dotenv_values("../database/.env")

"""
With the schemas and client out of the way, we can begin to use them
within our routers. Let's start with the write router. This router
will include the following endpoint: `/write/<bucket>/insert`.

The insert endpoint will take a `InsertWaveHeightRequest` request
from the caller. It will instantiate the client and pass the
location and height from the client request to the `record_wave_height`
method. Then, it will just return the stored data to the user.
"""
write_router = APIRouter(prefix="/write")


@write_router.post(
    "/{bucket}/insert",
    summary="Insert data into a bucket.",
    responses={
        201: {"description": "Successfully Inserted Into Bucket."},
        400: {"description": "Bad data requested."},
        404: {"description": "Bucket not found."},
        503: {"description": "InfluxDB Not Available"},
    },
)
async def insert_bucket(
    r: InsertWaveHeightRequest, bucket: str
) -> InsertWaveHeightResponse:
    logger.debug(f"Insert data into {bucket=}")
    ic = InfluxWaveClient(
        env_values.get('BUCKET_NAME'), 
        env_values.get('INFLUXDB_TOKEN'), 
        env_values.get('ORGANISATION'), 
        f"http://localhost:{env_values.get('INFLUXDB_PORT')}"
    )
    try:
        await ic.record_wave_height(r.location, r.height)
    except (
        InfluxNotAvailableException,
        BucketNotFoundException,
        BadQueryException,
    ) as e:
        raise HTTPException(
            status_code=e.STATUS_CODE,
            detail=e.DESCRIPTION,
        )
    logger.debug(f"Inserted data into {bucket=} with {r.location=} and {r.height=}")
    return InsertWaveHeightResponse(bucket=bucket, location=r.location, height=r.height)