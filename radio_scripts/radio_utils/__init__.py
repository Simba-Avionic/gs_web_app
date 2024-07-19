import serial  # pip install pyserial
import time
import serial.tools.list_ports
import re

def detect_baud_rate(port):
    """
    Detects the baud rate of the connected SiK radio.

    Args:
        port (str): The serial port to check.

    Returns:
        int: The detected baud rate, or None if detection failed.
    """
    baud_rates = [1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200]
    for baud in baud_rates:
        print("Checking: " + str(baud))
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                # Clear the buffer
                ser.flushInput()
                ser.flushOutput()
                # Enter AT command mode
                time.sleep(1)
                ser.write(b'+++')
                time.sleep(1)
                ser.write(b'ATI\r\n')
                time.sleep(1)
                response = ser.read_all().decode(errors='ignore').strip()
                if 'SiK' in response:
                    return baud
        except serial.SerialException:
            pass
    return None

def list_serial_ports():
    """
    Lists available serial port names.

    Returns:
        list: List of available serial port names.
    """
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def pick_pickables():
    """
    Prompts the user to select a serial port and enter or detect a baud rate.

    Returns:
        tuple: Selected serial port and baud rate, or None if the selection failed.
    """
    ports = list_serial_ports()
    if not ports:
        print("No serial ports found.")
        return

    print("Available serial ports:")
    for i, port in enumerate(ports):
        print(f"{i}: {port}")

    # Prompt user to select a serial port
    port_index = int(input("Select the serial port index: "))
    if port_index < 0 or port_index >= len(ports):
        print("Invalid index.")
        return

    selected_port = ports[port_index]

    # Prompt user for baud rate or attempt to detect automatically
    detected_baud = input("Insert baud rate (press enter if not sure): ").strip()
    if detected_baud == '':
        detected_baud = detect_baud_rate(selected_port)
        if detected_baud:
            print(f"Successfully detected baud rate: {detected_baud}")
        else:
            print("Failed to detect baud rate.")
            return
    else:
        detected_baud = int(detected_baud)

    return selected_port, detected_baud

def display_parameters_descriptions():
    description = """
    S0:FORMAT=<value>
    Command: ATS0?
    Description: The data format used by the radio.

    S1:SERIAL_SPEED=<value>
    Command: ATS1?
    Description: The baud rate for communication between the radio and the connected device (e.g., 4, 8, 16, 32, 64, 128 kbps).

    S2:AIR_SPEED=<value>
    Command: ATS2?
    Description: The data transmission rate over the air between radios (e.g., 4, 8, 16, 32, 64, 128 kbps).

    S3:NETID=<value>
    Command: ATS3?
    Description: Identifies the network, allowing multiple sets of radios to operate independently in the same area (e.g., 0-255).

    S4:TXPOWER=<value>
    Command: ATS4?
    Description: The transmit power level of the radio in dBm.

    S5:ECC=<value>
    Command: ATS5?
    Description: Enables or disables Error Correction Coding (ECC) to improve data reliability (0 for off, 1 for on).

    S6:MAVLINK=<value>
    Command: ATS6?
    Description: Optimizes the radio for MAVLink data transmission (0=no MAVLink framing, 1=frame mavlink, 2=low latency mavlink).

    S7:OPPRESEND=<value>
    Command: ATS7?
    Description: OppResend parameter, related to packet retransmission.

    S8:MIN_FREQ=<value>
    Command: ATS8?
    Description: Sets the minimum operating frequency of the radio.

    S9:MAX_FREQ=<value>
    Command: ATS9?
    Description: Sets the maximum operating frequency of the radio.

    S10:NUM_CHANNELS=<value>
    Command: ATS10?
    Description: The number of frequency channels used by the radio.

    S11:DUTY_CYCLE=<value>
    Command: ATS11?
    Description: Limits the percentage of time the radio spends transmitting (0-100).

    S12:LBT_RSSI=<value>
    Command: ATS12?
    Description: Listen Before Talk (LBT) Received Signal Strength Indicator (RSSI) threshold.

    S13:MANCHESTER=<value>
    Command: ATS13?
    Description: Enables or disables Manchester encoding to reduce errors (0 for off, 1 for on).

    S14:RTSCTS=<value>
    Command: ATS14?
    Description: Enables or disables RTS/CTS hardware flow control (0 for off, 1 for on).

    S15:MAX_WINDOW=<value>
    Command: ATS15?
    Description: Sets the maximum window size for packet transmission.
    """
    print(description)

def display_ATI_commands():
    ATI_commands = """
    The AT commands available are:
    ATI - show radio version
    ATI2 - show board type
    ATI3 - show board frequency
    ATI4 - show board version
    ATI5 - show all user settable EEPROM parameters
    ATI6 - display TDM timing report
    ATI7 - display RSSI signal report
    ATO - exit AT command mode
    ATSn? - display radio parameter number 'n'
    ATSn=X - set radio parameter number 'n' to 'X'
    ATZ - reboot the radio
    AT&W - write current parameters to EEPROM
    AT&F - reset all parameters to factory default
    AT&T=RSSI - enable RSSI debug reporting
    AT&T=TDM - enable TDM debug reporting
    AT&T - disable debug reporting"""
    print(ATI_commands)

class RadioModule(serial.Serial):
    def __init__(self, serial_port, baud_rate, timeout=1):
        super().__init__(port=serial_port, baudrate=baud_rate, timeout=timeout)
        self.command_mode_active = False
        # useful parent funcs
        # .read_all()
        # .write()
    def __del__(self) -> None:
        self.leave_command_mode()
        return super().__del__()

    ## setters ##
    def send_at_command(self, command):
        pattern = re.compile(r'^ATI(2|3|4|5|6|7)?$|^ATO$|^ATS\d+(\?|(=\d+))$|^ATZ$|^AT&W$|^AT&F$|^AT&T(=RSSI|=TDM)?$')
        if not pattern.match(command):
            print('Invalid AT command: ' + command)
            display_ATI_commands()
            return ''
        self.write(command.encode() + b'\r\n')
        time.sleep(1)
        response = self.read_all().decode(errors='ignore').strip()
        return response

    def enter_command_mode(self):
        if self.command_mode_active or ('SiK' in self.send_at_command('ATI')):
            print("Already in command mode")
            self.command_mode_active = True
            return True
        for _ in range(3):  # Try multiple times
            self.flushInput()
            self.flushOutput()
            time.sleep(1)
            self.write(b'+++')
            time.sleep(1)
            if 'SiK' in self.send_at_command('ATI'):
                self.command_mode_active = True
                return True
        return False

    def leave_command_mode(self):
        if not self.command_mode_active:
            return True
        self.send_at_command('ATO')
        self.command_mode_active = False

    def set_transmit_power(self, power):
        valid_powers = [1, 2, 5, 8, 11, 14, 17, 20]
        if power not in valid_powers:
            print(f"Invalid power level: {power}. Valid levels: {valid_powers}")
            return

        if self.enter_command_mode():
            response = self.send_at_command(f'ATS4={power}')
            if 'OK' in response:
                if 'OK' in self.send_at_command('AT&W'):
                    print('Successfully set and saved transmit power to EEPROM.')
                else:
                    print('Failed to save to EEPROM.')
            else:
                print('Failed to set transmit power.')
            self.leave_command_mode()
        else:
            print("Failed to enter command mode")

    ## Parsing Methods ##
    def parse_ati7_response(self, response):
        """
        Parses the response from the ATI7 command to extract RSSI and other diagnostics.

        Args:
            response (str): The response string from the ATI7 command.

        Returns:
            dict: A dictionary containing the parsed values.
        """
        result = {
            'L_RSSI': None,
            'R_RSSI': None,
            'L_noise': None,
            'R_noise': None,
            'packets': None,
            'tx_errors': None,
            'rx_errors': None,
            'successful_tx': None,
            'successful_rx': None,
            'ecc_corrected': None,
            'ecc_uncorrected': None,
            'temperature': None,
            'dco': None
        }
        
        match = re.search(r'L/R RSSI: (\d+)/(\d+)', response)
        if match:
            result['L_RSSI'] = int(match.group(1))
            result['R_RSSI'] = int(match.group(2))

        match = re.search(r'L/R noise: (\d+)/(\d+)', response)
        if match:
            result['L_noise'] = int(match.group(1))
            result['R_noise'] = int(match.group(2))

        match = re.search(r'pkts: (\d+)', response)
        if match:
            result['packets'] = int(match.group(1))

        match = re.search(r'txe=(\d+)', response)
        if match:
            result['tx_errors'] = int(match.group(1))

        match = re.search(r'rxe=(\d+)', response)
        if match:
            result['rx_errors'] = int(match.group(1))

        match = re.search(r'stx=(\d+)', response)
        if match:
            result['successful_tx'] = int(match.group(1))

        match = re.search(r'srx=(\d+)', response)
        if match:
            result['successful_rx'] = int(match.group(1))

        match = re.search(r'ecc=(\d+)/(\d+)', response)
        if match:
            result['ecc_corrected'] = int(match.group(1))
            result['ecc_uncorrected'] = int(match.group(2))

        match = re.search(r'temp=(-?\d+)', response)
        if match:
            result['temperature'] = int(match.group(1))

        match = re.search(r'dco=(\d+)', response)
        if match:
            result['dco'] = int(match.group(1))

        return result
    
    def parse_ati6_response(self, response):
        """
        Parses the response from the ATI6 command to extract TDM timing report details.

        Args:
            response (str): The response string from the ATI6 command.

        Returns:
            dict: A dictionary containing the parsed values.
        """
        result = {
            'silence_period': None,
            'tx_window_width': None,
            'max_data_packet_length': None
        }
        
        match = re.search(r'silence_period: (\d+)', response)
        if match:
            result['silence_period'] = int(match.group(1))

        match = re.search(r'tx_window_width: (\d+)', response)
        if match:
            result['tx_window_width'] = int(match.group(1))

        match = re.search(r'max_data_packet_length: (\d+)', response)
        if match:
            result['max_data_packet_length'] = int(match.group(1))

        return result
    
    ## getters ##
    def display_current_parameters(self):
        if self.enter_command_mode():
            response = self.send_at_command('ATI5')
            print(response)
            self.leave_command_mode()
        else:
            print("Failed to enter command mode")
    
    def get_output_data(self):
        if self.enter_command_mode():
            tdm_report = self.parse_ati6_response(self.send_at_command('ATI6'))
            rssi_report = self.parse_ati7_response(self.send_at_command('ATI7'))
            return tdm_report, rssi_report
        else:
            print("Failed to enter command mode")
            return

if __name__ == '__main__':
    # selected_port, detected_baud = pick_pickables()
    selected_port ='COM5'
    detected_baud = 57600
    if selected_port and detected_baud:
        radio = RadioModule(selected_port, detected_baud)
        radio.display_current_parameters()
        # Change power as needed
        # radio.set_transmit_power(20)  # Example power level to set
        tdm_report, rssi_report = radio.get_output_data()
        print(rssi_report)
        print(tdm_report)
        radio.leave_command_mode()
