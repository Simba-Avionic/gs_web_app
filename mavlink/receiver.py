from pymavlink import mavutil
import os

os.environ["MAVLINK_DIALECT"] = "simba"

class MavlinkReceiver:
    def __init__(self, port):
        self.master = mavutil.mavlink_connection(port, baud=57600, dialect="simba")
        print("Mavlink connection established. Heartbeat received. Waiting for custom messages...")
        self.receive_mav_msgs()

    def receive_mav_msgs(self):
        while True:
            msg = self.master.recv_match(blocking=True)
            if not msg:
                continue

            msg_type = msg.get_type()
            print(f"Received MAVLink message: {msg_type}")

            for field_name in msg.get_fieldnames():
                field_value = getattr(msg, field_name, None)
                print(f"  {field_name}: {field_value}")

if __name__ == "__main__":
    MavlinkReceiver(port="/dev/ttyUSB0")  # Replace with your serial port