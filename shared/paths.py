import os

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))

ROOT_DIR = os.path.dirname(CURRENT_DIR)

CONFIG_JSON_PATH = os.path.join(ROOT_DIR, "config.json")
SIMBA_XML_PATH = os.path.join(ROOT_DIR, "mavlink", "simba_mavlink", "simba.xml")
TILES_DIRECTORY = os.path.join(ROOT_DIR, "frontend", "public", "tiles")