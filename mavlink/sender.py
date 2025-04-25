from pymavlink import mavutil
import os
import time
import random
import json
from parser import parse_simba_xml

SIMBA_XML_PATH = "message_definitions/v1.0/simba.xml"
CONFIG_PATH = "config.json"

os.environ["MAVLINK_DIALECT"] = "simba"

class MavlinkSender:
    def __init__(self, port):
        self.master = mavutil.mavlink_connection(port, baud=57600, dialect="simba")
        self.messages = parse_simba_xml(SIMBA_XML_PATH)
        self.config = self.load_config(CONFIG_PATH)

        self.send_messages()

    def load_config(self, config_path):
        """Load the configuration file."""
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Configuration file not found: {config_path}")
        
        with open(config_path, "r") as f:
            config = json.load(f)
        
        print(f"Loaded configuration: {config}")
        return config

    def encode_message(self, msg):
        fields = msg["fields"]
        args = []

        for field in fields:
            if field["type"] == "uint64_t":
                args.append(int(time.time() * 1e6))
            elif field["type"] in ["uint8_t", "uint16_t", "uint32_t", "int32_t"]:
                args.append(random.randint(0, 100))
            elif field["type"] == "float":
                args.append(random.random())
            else:
                args.append(0)
        return args

    def send_message(self, msg_name, args):
        try:
            send_func = getattr(self.master.mav, f"{msg_name.lower()}_send")
            send_func(*args)
            print(f"Sent message: {msg_name} with args: {args}")
        except AttributeError:
            print(f"Message sending function not found for: {msg_name}")
        except Exception as e:
            print(f"Error sending message {msg_name}: {e}")

    def send_messages(self):
        last_sent = {msg["name"]: 0 for msg in self.messages}

        while True:
            current_time = time.time() * 1e3

            for msg in self.messages:
                msg_name = msg["name"]

                if msg_name not in self.config:
                    continue

                interval = self.config[msg_name].get("interval", 1000)  # Default interval is 1000ms
                if current_time - last_sent[msg_name] < interval:
                    continue
                
                args = self.encode_message(msg)
                self.send_message(msg_name, args)
                last_sent[msg_name] = current_time

            time.sleep(0.01)

if __name__ == "__main__":
    MavlinkSender(port='/dev/ttyUSB1')