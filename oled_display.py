from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import sh1106
from PIL import ImageFont
import socket
import subprocess
import time

import netifaces

def get_ip():
    try:
        output = subprocess.check_output(["hostname", "-I"])
        ip_list = output.decode().strip().split()
        for ip in ip_list:
            if not ip.startswith("127."):
                return ip
        return "---"
    except Exception:
        return "---"


def is_simba_app_active():
    try:
        subprocess.check_call(["systemctl", "is-active", "--quiet", "simba-app"])
        return "App: ON"
    except subprocess.CalledProcessError:
        return "App: OFF"

def check_docker_container(container_name, display_name):
    try:
        output = subprocess.check_output(
            ["docker", "ps", "--filter", f"name={container_name}", "--filter", "status=running", "--format", "{{.Names}}"]
        )
        return f"{display_name}: ON" if container_name.strip().encode() in output.splitlines() else f"{display_name}: OFF"
    except Exception:
        return f"{display_name}: OFF"

# Initialize OLED display (flipped upside down)
serial = i2c(port=1, address=0x3C)
device = sh1106(serial, rotate=2)

while True:
    ip = get_ip()
    simba_status = is_simba_app_active()
    influx_status = check_docker_container("influxdb", "Database")
    grafana_status = check_docker_container("grafana", "Grafana")

    with canvas(device) as draw:
        # draw.rectangle(device.bounding_box, outline="white", fill="black")
        draw.text((4, 0), f"IP: {ip}", fill="white")
        draw.text((4, 16), simba_status, fill="white")
        draw.text((4, 32), influx_status, fill="white")
        draw.text((4, 48), grafana_status, fill="white")

    time.sleep(1)