import os
import shutil
import requests
import subprocess
from requests.auth import HTTPDigestAuth
from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse, JSONResponse
from loguru import logger
from pydantic import BaseModel
from datetime import datetime
from dotenv import load_dotenv, find_dotenv

load_dotenv(find_dotenv())

# ------------------ CONFIG & STATE ------------------
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

processes = {
    cam_id: {"stream": None, "recording": None, "current_file": None}
    for cam_id in CAMERAS
}

for cam in CAMERAS.values():
    os.makedirs(cam["hls_dir"], exist_ok=True)
    os.makedirs(cam["recordings_dir"], exist_ok=True)

class PTZCommand(BaseModel):
    pan: int; tilt: int; zoom: int; speed: int

# ------------------ CORE HELPERS ------------------

def get_camera(cam_id: str):
    if cam_id not in CAMERAS:
        raise HTTPException(status_code=404, detail="Unknown camera")
    return CAMERAS[cam_id], processes[cam_id]

def _manage_ffmpeg_stream(cam_id: str):
    """
    Centralized function to start FFmpeg. 
    Handles cleanup of old files and connection timeouts.
    """
    cam, proc = get_camera(cam_id)
    hls_file = os.path.join(cam["hls_dir"], "stream.m3u8")

    # Start if process is missing or has crashed/stopped
    if not proc["stream"] or proc["stream"].poll() is not None:
        logger.info(f"Starting FFmpeg for {cam_id}")

        # Clean old HLS segments so the web app doesn't load 'ghost' footage
        if os.path.exists(cam["hls_dir"]):
            for f in os.listdir(cam["hls_dir"]):
                if f.endswith(('.ts', '.m3u8')):
                    try: os.remove(os.path.join(cam["hls_dir"], f))
                    except: pass

        proc["stream"] = subprocess.Popen(
            [
                "ffmpeg",
                "-rtsp_transport", "tcp",
                "-stimeout", "5000000", # 5s timeout to detect unplugged cable
                "-i", cam["rtsp"],
                "-c:v", "copy",
                "-c:a", "aac",
                "-b:a", "128k",
                "-f", "hls",
                "-hls_time", "1",
                "-hls_list_size", "5",
                "-hls_flags", "delete_segments", # No append_list to ensure fresh start
                hls_file,
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    return hls_file

def start_all_camera_streams():
    """Helper used during app startup."""
    for cam_id in CAMERAS:
        _manage_ffmpeg_stream(cam_id)

# ------------------ ROUTER ------------------
router = APIRouter()

@router.get("/camera/{cam_id}/stream.m3u8")
def get_camera_stream(cam_id: str):
    hls_file = _manage_ffmpeg_stream(cam_id)

    if not os.path.isfile(hls_file):
        return JSONResponse(
            status_code=202,
            content={"status": "starting", "detail": "Stream preparing. Retry shortly."},
        )

    return FileResponse(
        hls_file,
        media_type="application/vnd.apple.mpegurl",
        headers={
            "Access-Control-Allow-Origin": "*",
            "Cache-Control": "no-cache, no-store, must-revalidate" # Force browser to check for new playlist
        },
    )

@router.post("/ptz/{cam_id}/move")
def ptz_move(cam_id: str, cmd: PTZCommand):
    cam, _ = get_camera(cam_id)
    url = f"http://{cam['ip']}/ISAPI/PTZCtrl/channels/1/continuous"
    try:
        response = requests.put(
            url,
            headers={"Content-Type": "application/xml"},
            data=f"<PTZData><pan>{cmd.pan}</pan><tilt>{cmd.tilt}</tilt><zoom>{cmd.zoom}</zoom><speed>{cmd.speed}</speed></PTZData>",
            auth=HTTPDigestAuth("admin", "simba123"),
            timeout=3 
        )
        response.raise_for_status()
        return {"message": "PTZ command sent"}
    except Exception as e:
        logger.error(f"PTZ Error for {cam_id}: {str(e)}")
        raise HTTPException(status_code=502, detail="Camera communication error")

@router.post("/camera/{cam_id}/start_recording")
def start_recording(cam_id: str):
    cam, proc = get_camera(cam_id)
    if proc["recording"]:
        return {"status": "already recording"}

    filename = os.path.join(cam["recordings_dir"], f"{cam_id}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4")
    proc["current_file"] = filename
    proc["recording"] = subprocess.Popen([
        "ffmpeg", "-rtsp_transport", "tcp", "-i", cam["rtsp"],
        "-c:v", "copy", "-c:a", "aac", filename
    ])
    return {"status": "recording started", "file": filename}

@router.post("/camera/{cam_id}/stop_recording")
def stop_recording(cam_id: str):
    _, proc = get_camera(cam_id)
    if proc["recording"]:
        proc["recording"].terminate()
        proc["recording"].wait()
        proc["recording"] = None
        return {"status": "recording stopped", "file": proc["current_file"]}
    return {"status": "not recording"}

@router.get("/camera/{cam_id}/download_recording")
def download_recording(cam_id: str):
    _, proc = get_camera(cam_id)
    file_path = proc.get("current_file")
    if not file_path or not os.path.exists(file_path):
        return JSONResponse(status_code=204, content={"detail": "Not ready"})
    return FileResponse(file_path, media_type="video/mp4")