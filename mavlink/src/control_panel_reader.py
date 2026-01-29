import serial
import glob
import time
import xml.etree.ElementTree as ET

class ControlPanelReader:

    SWITCH_ACTIONS = {
        ("valve_feed_oxidizer", "gs"): 0,       # bit 0
        ("valve_vent_oxidizer", "gs"): 0,       # bit 1
        ("decoupler_oxidizer", "gs"): 0,        # bit 2
        ("valve_feed_pressurizer", "gs"): 0,    # bit 3
        ("valve_vent_pressurizer", "gs"): 0,    # bit 4
        ("decoupler_pressurizer", "gs"): 0,     # bit 5
        ("arm_disarm", "rocket"): 0,            # bit 6
        ("tank_vent", "rocket"): 0,             # bit 7
        ("ignition", "rocket"): 0,              # bit 8
        ("abort", "abort"): 0,                  # bit 9
        ("dump", "rocket"): 0,                  # bit 10
        ("enable_cameras", "rocket"): 0,        # bit 11
        ("tare_rocket", "gs"): 0,               # bit 12
        ("tare_oxidizer", "gs"): 0,             # bit 13
        ("tare_pressurizer", "gs"): 0,          # bit 14
    }

    def __init__(self, port=None, baudrate=57600, timeout=0.1, num_retries=3, mavlink_xml_path="simba_mavlink/simba.xml"):
        self.baudrate = baudrate
        self.num_retries = num_retries
        self.ser = None
        self.thread = None

        self.mavlink_xml_path = mavlink_xml_path
        self.mavlink_gs_flags = self._parse_mavlink_flags("SIMBA_GS_FLAGS") 
        self.latest_switches = {}
        self.switch_to_mavlink = {
            ("arm_disarm", "rocket"): "SIMBA_GS_ARM",
            ("ignition", "rocket"): "SIMBA_GS_LAUNCH",
            ("abort", "abort"): "SIMBA_GS_ABORT",
            ("tank_vent", "rocket"): "SIMBA_GS_VENT_VALVE",
            ("dump", "rocket"): "SIMBA_GS_DUMP_VALVE",
            ("enable_cameras", "rocket"): "SIMBA_GS_CAMERAS",
            # Add more if needed; GS-specific switches can map here too
        }

        if port:
            self.ser = serial.Serial(port, baudrate)
            print(f"Using specified Control Panel port: {port} | baud: {baudrate}")
        else:
            self.scan_and_connect()

    def _parse_mavlink_flags(self, enum_name):
        """Parse MAVLink XML to extract flag values."""
        tree = ET.parse(self.mavlink_xml_path)
        root = tree.getroot()
        flags = {}
        for enum in root.findall("enums/enum"):
            if enum.get("name") == enum_name:
                for entry in enum.findall("entry"):
                    flags[entry.get("name")] = int(entry.get("value"))
        return flags

    def scan_and_connect(self):
        available_ports = sorted(glob.glob('/dev/ttyACM*'))

        if not available_ports:
            raise serial.SerialException("No serial ports found")

        print(f"Found {len(available_ports)} potential serial ports")

        for port in available_ports:
            try:
                print(f"Trying port {port}...")
                test_ser = serial.Serial(port, self.baudrate, timeout=0.1)
                time.sleep(1)  # Allow time for Arduino reset after connection

                for _ in range(self.num_retries):
                    raw = test_ser.readline().decode().strip()
                    if raw:
                        try:
                            # Validate data format (expecting uint16_t in decimal form)
                            val = int(raw)
                            if 0 <= val <= 65535:
                                print(f"Found active control panel on {port}: {val}")
                                self.ser = test_ser
                                return
                        except ValueError:
                            # Not a valid integer
                            pass
                    time.sleep(0.1)

                test_ser.close()

            except (serial.SerialException, OSError) as e:
                print(f"Error on port {port}: {e}")
        print("Couldn't find a control panel port!!!")

    def read_switches(self):
        """
        Read switch states and return in SWITCH_ACTIONS format.

        Returns:
            Dictionary with (name, category) tuples as keys and states (0 or 1) as values
        """
        actions = self.SWITCH_ACTIONS.copy()

        if not self.ser:
            return None

        try:
            switch_value = self.ser.readline().decode(errors="ignore").strip() # Read a line and decode
            if switch_value == '':
                return

            switch_value = int(switch_value)
            
            # Map each bit to the corresponding key in SWITCH_ACTIONS
            for i, key in enumerate(actions.keys()):
                if i < 16:  # Only 16 bits in uint16_t
                    actions[key] = (switch_value >> i) & 1

            return actions
        except Exception as e:
            print(f"Error reading switches: {e}")
            return actions

    def get_rocket_actions(self):
        """
        Get only rocket-related actions.

        Returns:
            Dictionary of rocket-related actions
        """
        actions = self.read_switches()
        if actions is None:
            return None
        return {key: value for key, value in actions.items()
                if key[1] == "rocket"}

    def get_gs_actions(self):
        """
        Get GS/abort actions as MAVLink bitmask for SIMBA_GS_HEARTBEAT.

        Returns:
            Integer bitmask (for MAVLink).
        """
        actions = self.read_switches()
        if actions is None:
            return 0
        bitmask = 0
        for key, state in actions.items():
            if key in self.switch_to_mavlink and state == 1:
                flag_name = self.switch_to_mavlink[key]
                if flag_name in self.mavlink_gs_flags:
                    bitmask |= self.mavlink_gs_flags[flag_name]
        return bitmask

    def close(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)

        if self.ser:
            self.ser.close()
            self.ser = None

if __name__ == "__main__":
    reader = ControlPanelReader("/dev/ttyACM1", baudrate=57600)
    try:
        while True:
            actions = reader.read_switches()
            print(actions)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        reader.close()
        pass