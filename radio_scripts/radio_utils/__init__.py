import serial # pip install pyserial
import time
import serial.tools.list_ports
 
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