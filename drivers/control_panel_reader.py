import glob
import time
import serial
import threading
from loguru import logger

class ControlPanelReader:

    # For exact bit structure look into https://github.com/Simba-Avionic/control_panel_software
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

    def __init__(self, port=None, baudrate=57600, timeout=0.1, num_retries=3):
            self.baudrate = baudrate
            self.num_retries = num_retries
            self.ser = None
            self.thread = None
            self.running = False
            
            self.latest_switches = self.SWITCH_ACTIONS.copy()
            self._callbacks = []

            if port:
                self.ser = serial.Serial(port, baudrate, timeout=timeout)
                logger.info(f"Using specified Control Panel port: {port} | baud: {baudrate}")
            else:
                self.scan_and_connect()

    def register_callback(self, callback_func):
        self._callbacks.append(callback_func)

    def start_listening(self):
        if not self.ser:
            logger.error("Cannot start listening: No serial connection.")
            return

        self.running = True
        self.thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.thread.start()
        logger.info("Control Panel reader background thread started.")

    def _listen_loop(self):
        while self.running:
            new_actions = self.read_switches()
            if new_actions is not None:
                if new_actions != self.latest_switches:
                    self.latest_switches = new_actions.copy()
                    
                    for callback in self._callbacks:
                        try:
                            callback(self.latest_switches)
                        except Exception as e:
                            logger.error(f"Error in callback execution: {e}")
            time.sleep(0.05)

    def get_rocket_actions(self, actions=None):
        if actions is None: return None
        return {key: value for key, value in actions.items() if key[1] == "rocket" or key[1] == "abort"}

    def get_gs_actions(self, actions=None):
        if actions is None: return None
        return {key: value for key, value in actions.items() if key[1] == "gs" or key[1] == "abort"}

    def close(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.ser:
            self.ser.close()
            self.ser = None

    def scan_and_connect(self):
        available_ports = sorted(glob.glob('/dev/ttyACM*'))

        if not available_ports:
            raise serial.SerialException("No serial ports found")

        logger.info(f"Found {len(available_ports)} potential serial ports")

        for port in available_ports:
            try:
                logger.info(f"Trying port {port}...")
                test_ser = serial.Serial(port, self.baudrate, timeout=0.1)
                time.sleep(1)  # Allow time for Arduino reset after connection

                for _ in range(self.num_retries):
                    raw = test_ser.readline().decode().strip()
                    if raw:
                        try:
                            # Validate data format (expecting uint16_t in decimal form)
                            val = int(raw)
                            if 0 <= val <= 65535:
                                logger.info(f"Found active control panel on {port}: {val}")
                                self.ser = test_ser
                                return
                        except ValueError:
                            # Not a valid integer
                            pass
                    time.sleep(0.1)

                test_ser.close()

            except (serial.SerialException, OSError) as e:
                logger.error(f"Error on port {port}: {e}")
        
        logger.warning("Couldn't find a control panel port!")

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
            logger.error(f"Error reading switches: {e}")
            return actions

if __name__ == "__main__":
    reader = ControlPanelReader("/dev/ttyACM1", baudrate=57600)
    try:
        while True:
            logger.debug(reader.read_switches())
            time.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("Exiting...")
    finally:
        reader.close()
        pass