import serial
import glob
import time

class ControlPanelReader:

    SWITCH_ACTIONS = {
        0: "arm_disarm",
        1: "ignition",
        2: "abort",
        3: "tank_vent",
        4: "main_valve",
        5: "hose_vent",
        6: "decoupler",
        7: "N2O_He_switch"
    }

    def __init__(self, port=None, baudrate=9600, num_retries=3):
        self.baudrate = baudrate
        self.num_retries = num_retries
        self.ser = None
        
        if port:
            self.ser = serial.Serial(port, baudrate, timeout=1)
        else:
            self.scan_and_connect()

        self.start_reading()
    
    def scan_and_connect(self):
        available_ports = sorted(glob.glob('/dev/ttyACM*'))
        
        if not available_ports:
            raise serial.SerialException("No serial ports found")
        
        print(f"Found {len(available_ports)} potential serial ports")
        
        for port in available_ports:
            try:
                print(f"Trying port {port}...")
                test_ser = serial.Serial(port, self.baudrate, timeout=0.5)
                time.sleep(1)  # Allow time for Arduino reset after connection
                
                for _ in range(self.num_retries):
                    raw = test_ser.readline().decode().strip()
                    if raw:
                        try:
                            # Validate data format (expecting string of 0s and 1s)
                            if self._is_valid_binary_string(raw):
                                print(f"Found active control panel on {port}: {raw}")
                                self.ser = test_ser
                                return
                        except Exception:
                            pass
                    time.sleep(0.1)
                
                test_ser.close()
                
            except (serial.SerialException, OSError) as e:
                print(f"Error on port {port}: {e}")
        
        raise serial.SerialException("No active control panel found on any port")

    def _is_valid_binary_string(self, data):
        """Check if the data is a valid binary string (only 0s and 1s)."""
        return all(bit in '01' for bit in data)

    def read_switches(self):
        """
        Read switch states from the control panel.
        
        Returns:
            Dictionary where keys are switch positions (0-indexed) and values are states (0 or 1)
        """
        if not self.ser:
            return {}
            
        try:
            raw = self.ser.readline().decode().strip()
            if not raw or not self._is_valid_binary_string(raw):
                return {}
            
            # Convert the binary string to a dictionary
            # Example: "0101" becomes {0: 0, 1: 1, 2: 0, 3: 1}
            switch_states = {}
            for i, state in enumerate(raw):
                switch_states[i] = int(state)
                
            return switch_states
        except Exception as e:
            print(f"Error reading switches: {e}")
            return {}
        
    def _read_loop(self):
        while self.running and self.ser:
            try:
                switches = self.read_switches()
                if switches:
                    self.latest_switches = switches
                time.sleep(0.5)  # 50ms delay between reads
            except Exception as e:
                print(f"Error in read loop: {e}")
                time.sleep(1)  # Longer delay after error
    
    def get_switch_states(self):
        return self.latest_switches.copy()
    
    def get_actions(self):
        """
        Get the current actions based on switch positions with meaningful names.
        
        Returns:
            Dictionary of {action_name: state} where state is 0 or 1
        """
        actions = {}
        for pos, name in self.SWITCH_ACTIONS.items():
            # Get switch state (0 or 1) for this position, default to 0 if not found
            state = self.latest_switches.get(pos, 0)
            actions[name] = state
            
        return actions
    
    def close(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
            
        if self.ser:
            self.ser.close()
            self.ser = None