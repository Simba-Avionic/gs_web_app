import serial
import glob
import time

class ControlPanelReader:

    SWITCH_ACTIONS = {
        ("decoupler", "gs"): 0,    # bit 0
        ("main_valve", "gs"): 0,      # bit 1
        ("hose_vent", "gs"): 0,     # bit 2
        ("tank_vent", "gs"): 0,        # bit 3
        ("ignition", "rocket"): 0,         # bit 4
        ("abort", "abort"): 0,         # bit 5
        ("arm_disarm", "rocket"): 0,     # bit 6
        ("N2O_He_switch", "gs"): 0           # bit 7
    }

    def __init__(self, port=None, baudrate=57600,timeout=0.1, num_retries=3):
        self.baudrate = baudrate
        self.num_retries = num_retries
        self.ser = None
        self.thread = None

        self.latest_switches = {}

        if port:
            self.ser = serial.Serial(port, baudrate)
            print(f"Using specified Control Panel port: {port} | baud: {baudrate}")
        else:
            self.scan_and_connect()

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
                            # Validate data format (expecting string of 0s and 1s)
                            if self._is_valid_binary_string(raw):
                                print(
                                    f"Found active control panel on {port}: {raw}")
                                self.ser = test_ser
                                return
                        except Exception:
                            pass
                    time.sleep(0.1)

                test_ser.close()

            except (serial.SerialException, OSError) as e:
                print(f"Error on port {port}: {e}")

        raise serial.SerialException(
            "No active control panel found on any port")

    def _is_valid_binary_string(self, data):
        """Check if the data is a valid binary string (only 0s and 1s)."""
        return all(bit in '01' for bit in data)

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
            switch_value = int(self.ser.readline().decode().strip())  # Read a line and decode

            if switch_value == '':
                return
            
            # Map each bit to the corresponding key in SWITCH_ACTIONS
            for i, key in enumerate(actions.keys()):
                if i < 8:  # Only 8 bits in uint8_t
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
        Get only ground support-related actions.

        Returns:
            Dictionary of ground segment-related actions
        """
        actions = self.read_switches()
        if actions is None:
            return None
        return {key: value for key, value in actions.items()
                if key[1] == "gs"}

    def close(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)

        if self.ser:
            self.ser.close()
            self.ser = None

if __name__ == "__main__":
    reader = ControlPanelReader("/dev/ttyACM0", baudrate=57600)
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