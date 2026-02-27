from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import sh1106
from PIL import ImageFont
import socket
import subprocess
import time

import netifaces

def get_systemd_boot_info():
    try:
        output = subprocess.check_output(["systemd-analyze"], text=True).strip()
        # Split by '=' and take the last part, then strip whitespace
        if "=" in output:
            total_time = output.split("=")[-1].strip()
            return f"{total_time}"
        return ""
    except Exception:
        return ""

def get_device():
    for i in range(10):
        try:
            serial = i2c(port=1, address=0x3C)
            device = sh1106(serial, rotate=2)
            return device
        except Exception as e:
            print(f"OLED connection attempt {i+1} failed: {e}")
            time.sleep(3)
    return None

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


device = get_device()
boot_time = None

if device is None:
    print("Could not initialize OLED after multiple attempts. Exiting.")
    exit(1)


while True:
    ip = get_ip()
    simba_status = is_simba_app_active()
    influx_status = check_docker_container("influxdb", "Database")
    boot_time = get_systemd_boot_info()

    with canvas(device) as draw:
        draw.text((4, 0), f"IP: {ip}", fill="white")
        draw.text((4, 16), simba_status, fill="white")
        draw.text((4, 32), influx_status, fill="white")
        draw.text((4, 48), f"Boot  time: {boot_time}", fill="white")

    time.sleep(1)