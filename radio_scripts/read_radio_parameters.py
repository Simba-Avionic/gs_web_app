import radio_utils

def send_at_command(serial_conn, command, response_timeout=1):
    serial_conn.write(command.encode() + b'\r\n')
    radio_utils.time.sleep(response_timeout)
    response = serial_conn.read_all().decode().strip()
    return response

def prettify_response(response: str):
    # I know this is lazy, but what did you expected from python - efficiency?
    response = response.replace('OK\r\nATI5\r\n', '')
    response = response.replace('TXPOWER', 'TXPOWER (Transmit Power)\n(1=1.3milliWats, 2=1.5mW, 5=3.2mW, 8=6.3mW,11=12.5mW, 14=25mW, 17=50mW, 20=100mW)')
    response = response.replace('ECC', 'ECC (Error Correction Bool)')
    response = response.replace('MAVLINK', 'MAVLINK (0=Raw Data; 1=Mavlink; 2=Low Latency)')
    response = response.replace('OPPRESEND', 'OPPRESEND bool - attempt resend when error detected')
    response = response.replace('=', ' = ')
    response = response.replace(':', ': ')
    print('\n' + response)
    

def main():
    try:
        selected_port, detected_baud = radio_utils.pick_pickables()
        response = '+++ATI5'
        max_iter = 5
        i = 1
        while response == '+++ATI5' and i <= max_iter:
            print("Read attempt no. " + str(i))
            # Open the serial connection
            with radio_utils.serial.Serial(selected_port, detected_baud, timeout=3) as ser:
                # Enter AT command mode
                radio_utils.time.sleep(1)  # Ensure guard radio_utils.time
                ser.write(b'+++')
                radio_utils.time.sleep(1)  # Ensure guard time
                # Read parameters using ATI5
                response = send_at_command(ser, 'ATI5')
                # Exit AT command mode
                send_at_command(ser, 'ATO')
                i += 1
        
        print('\nRadio Parameters:')
        print(response)
        # prettify_response(response) # uncomment for more verbose / more explanatory definition of parameters

    except ValueError as ve:
        print(f'Error: Invalid input. {ve}')
    except radio_utils.serial.SerialException as se:
        print(f'Error: Serial port error. {se}')
    except Exception as e:
        print(f'Unexpected error: {e}')

if __name__ == '__main__':
    main()