from pydantic import BaseModel, Field
from typing import List


class InfluxNotAvailableException(Exception):
    STATUS_CODE = 503
    DESCRIPTION = "Unable to connect to influx."


class BucketNotFoundException(Exception):
    STATUS_CODE = 404
    DESCRIPTION = "Bucket Not Found."


class BadQueryException(Exception):
    STATUS_CODE = 400
    DESCRIPTION = "Bad Query."


# class InfluxWaveRecord(BaseModel):
#     location: str = Field(description="Location of the recorded wave")
#     height: float = Field(description="Height of the recorded wave")


# class InsertWaveHeightRequest(BaseModel):
#     location: str = Field(description="Location of the recorded wave")
#     height: float = Field(description="Height of the recorded wave")


# class InsertWaveHeightResponse(BaseModel):
#     location: str = Field(description="Location of the recorded wave")
#     height: float = Field(description="Height of the recorded wave")


# class ListBucketResponse(BaseModel):
#     records: List[InfluxWaveRecord] = Field(
#         description="Contents of the simba bucket"
#     )