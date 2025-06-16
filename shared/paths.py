import os

# You might need to adjust the path based on where you cloned the repo
ROOT_DIR = os.path.join(os.path.expanduser("~"), "Desktop/gs_web_app")

CONFIG_JSON_PATH = os.path.join(ROOT_DIR, "config.json")
SIMBA_XML_PATH = os.path.join(ROOT_DIR, "mavlink/simba_mavlink/simba.xml")
TILES_DIRECTORY = os.path.join(ROOT_DIR, "frontend/public/tiles")
