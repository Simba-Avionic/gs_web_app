import os
import subprocess
import requests
from requests.auth import HTTPDigestAuth
from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse, JSONResponse
from loguru import logger
from pydantic import BaseModel
from datetime import datetime
from dotenv import load_dotenv, find_dotenv

load_dotenv(find_dotenv())

# ------------------ CAMERA CONFIG ------------------
BASE_HLS_DIR = "temp"

CAMERAS = {
    "camera1": {
        "ip": os.getenv("CAMERA1_IP"),
        "rtsp": f"rtsp://admin:simba123@{os.getenv('CAMERA1_IP')}:554/Streaming/Channels/101",
        "hls_dir": "temp/camera1",
        "recordings_dir": "temp/camera1/recordings",
    },
    "camera2": {
        "ip": os.getenv("CAMERA2_IP"),
        "rtsp": f"rtsp://admin:simba123@{os.getenv('CAMERA2_IP')}:554/Streaming/Channels/101",
        "hls_dir": "temp/camera2",
        "recordings_dir": "temp/camera2/recordings",
    },
}

for cam in CAMERAS.values():
    os.makedirs(cam["hls_dir"], exist_ok=True)
    os.makedirs(cam["recordings_dir"], exist_ok=True)

# state per camera
processes = {
    cam_id: {"stream": None, "recording": None, "current_file": None}
    for cam_id in CAMERAS
}

# ------------------ MODELS ------------------
class PTZCommand(BaseModel):
    pan: int
    tilt: int
    zoom: int
    speed: int

# ------------------ HELPERS ------------------
def get_camera(cam_id: str):
    if cam_id not in CAMERAS:
        raise HTTPException(status_code=404, detail="Unknown camera")
    return CAMERAS[cam_id], processes[cam_id]

# in your cameras.py (where router is defined)

def start_all_camera_streams():
    for cam_id, cam in CAMERAS.items():
        hls_file = os.path.join(cam["hls_dir"], "stream.m3u8")
        proc = processes[cam_id]
        if not proc["stream"] or proc["stream"].poll() is not None:
            logger.info(f"Starting FFmpeg for {cam_id}")
            proc["stream"] = subprocess.Popen(
                [
                    "ffmpeg",
                    "-rtsp_transport", "tcp",
                    "-i", cam["rtsp"],
                    "-c", "copy",
                    "-f", "hls",
                    "-hls_time", "1",
                    "-hls_list_size", "5",
                    "-hls_flags", "delete_segments+append_list",
                    hls_file,
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )


# ------------------ ROUTER ------------------
router = APIRouter()

@router.post("/ptz/{cam_id}/move")
def ptz_move(cam_id: str, cmd: PTZCommand):
    cam, _ = get_camera(cam_id)
    url = f"http://{cam['ip']}/ISAPI/PTZCtrl/channels/1/continuous"
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
        raise HTTPException(status_code=response.status_code, detail=f"Failed PTZ: {response.text}")
    return {"message": "PTZ command sent"}

@router.get("/camera/{cam_id}/stream.m3u8")
def get_camera_stream(cam_id: str):
    cam, proc = get_camera(cam_id)
    hls_file = os.path.join(cam["hls_dir"], "stream.m3u8")

    # If the manifest file doesn't exist yet, start FFmpeg and return 202
    if not os.path.isfile(hls_file):
        # start FFmpeg if not already running
        if not proc["stream"] or proc["stream"].poll() is not None:
            logger.info(f"Starting FFmpeg for {cam_id}")
            proc["stream"] = subprocess.Popen(
                [
                    "ffmpeg",
                    "-rtsp_transport", "tcp",
                    "-i", cam["rtsp"],
                    "-c", "copy",
                    "-f", "hls",
                    "-hls_time", "1",
                    "-hls_list_size", "5",
                    "-hls_flags", "delete_segments+append_list",
                    hls_file,
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        # Tell the client the stream is being prepared (client should retry)
        return JSONResponse(
            status_code=202,
            content={"status": "starting", "detail": "Stream is being prepared. Retry shortly."},
        )

    # Serve the manifest with a CORS header as well (safe fallback)
    return FileResponse(
        hls_file,
        media_type="application/vnd.apple.mpegurl",
        headers={"Access-Control-Allow-Origin": "*"},
    )

@router.post("/camera/{cam_id}/start_recording")
def start_recording(cam_id: str):
    cam, proc = get_camera(cam_id)
    if proc["recording"]:
        return {"status": "already recording"}

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(cam["recordings_dir"], f"{cam_id}_{timestamp}.mp4")
    proc["current_file"] = filename

    proc["recording"] = subprocess.Popen([
        "ffmpeg",
        "-rtsp_transport", "tcp",
        "-i", cam["rtsp"],
        "-c", "copy",
        filename
    ])
    return {"status": "recording started", "file": filename}

@router.post("/camera/{cam_id}/stop_recording")
def stop_recording(cam_id: str):
    _, proc = get_camera(cam_id)
    if proc["recording"]:
        proc["recording"].terminate()
        proc["recording"] = None
        return {"status": "recording stopped", "file": proc["current_file"]}
    return {"status": "not recording"}

@router.get("/camera/{cam_id}/download_recording")
def download_recording(cam_id: str):
    _, proc = get_camera(cam_id)
    if not proc["current_file"] or not os.path.exists(proc["current_file"]):
        raise HTTPException(status_code=404, detail="No recording available")
    return FileResponse(
        proc["current_file"],
        media_type="video/mp4",
        filename=os.path.basename(proc["current_file"])
    )
