import os
import subprocess
import requests
from requests.auth import HTTPDigestAuth
from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse
from loguru import logger
from pydantic import BaseModel
import mimetypes

# ------------------ CAMERA CONFIG ------------------
CAMERA_RTSP_URL = "rtsp://admin:simba123@192.168.1.200:554/Streaming/Channels/101"
CAMERA_HLS_DIR = "temp/camera"
CAMERA_HLS_FILE = os.path.join(CAMERA_HLS_DIR, "stream.m3u8")
OUTPUT_FILE = "recording.mp4"

os.makedirs(CAMERA_HLS_DIR, exist_ok=True)

ffmpeg_process = None

mimetypes.add_type("application/vnd.apple.mpegurl", ".m3u8")
mimetypes.add_type("video/mp2t", ".ts")

recording_process = None

class PTZCommand(BaseModel):
    pan: int
    tilt: int
    zoom: int
    speed: int

def send_ptz_command(cmd: PTZCommand):
    url = f"http://192.168.1.200/ISAPI/PTZCtrl/channels/1/continuous"
    headers = {"Content-Type": "application/xml"}

    xml_body = f"""
    <PTZData>
      <pan>{cmd.pan}</pan>
      <tilt>{cmd.tilt}</tilt>
      <zoom>{cmd.zoom}</zoom>
      <speed>{cmd.speed}</speed>
    </PTZData>
    """

    response = requests.put(
        url,
        headers=headers,
        data=xml_body,
        auth=HTTPDigestAuth("admin", "simba123"),
    )
    if response.status_code != 200:
        raise HTTPException(
            status_code=response.status_code,
            detail=f"Failed to send PTZ command: {response.text}"
        )

def start_camera_stream():
    """Start FFmpeg process to convert RTSP to HLS for browser playback."""
    global ffmpeg_process
    logger.info(CAMERA_HLS_FILE)

    if ffmpeg_process and ffmpeg_process.poll() is None:
        logger.info("Camera streaming already running.")
        return

    logger.info(f"Starting FFmpeg for camera stream from {CAMERA_RTSP_URL}...")
    ffmpeg_process = subprocess.Popen(
        [
            "ffmpeg",
            "-rtsp_transport", "tcp",
            "-i", CAMERA_RTSP_URL,
            "-c", "copy",
            "-f", "hls",
            "-hls_time", "1",
            "-hls_list_size", "5",
            "-hls_flags", "delete_segments+append_list",
            CAMERA_HLS_FILE,
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

def stop_camera_stream():
    """Stop FFmpeg process."""
    global ffmpeg_process
    if ffmpeg_process:
        ffmpeg_process.terminate()
        ffmpeg_process.wait()
        ffmpeg_process = None
        logger.info("Camera stream stopped.")


# ---------------- FASTAPI ROUTER -----------------
router = APIRouter()

@router.post("/ptz/move")
def ptz_move(cmd: PTZCommand):
    send_ptz_command(cmd)
    return {"message": "PTZ command sent"}

@router.get("/camera/stream.m3u8")
def get_camera_stream():
    if not os.path.isfile(CAMERA_HLS_FILE):
        raise HTTPException(status_code=404, detail="Camera stream not ready")
    return FileResponse(CAMERA_HLS_FILE, media_type="application/vnd.apple.mpegurl")


@router.post("/start_recording")
def start_recording():
    global recording_process
    if recording_process:
        return {"status": "already recording"}
    recording_process = subprocess.Popen([
        "ffmpeg",
        "-rtsp_transport", "tcp",
        "-i", CAMERA_RTSP_URL,
        "-c", "copy",
        OUTPUT_FILE
    ])
    return {"status": "recording started"}

@router.post("/stop_recording")
def stop_recording():
    global recording_process
    if recording_process:
        recording_process.terminate()
        recording_process = None
        return {"status": "recording stopped"}
    return {"status": "not recording"}
