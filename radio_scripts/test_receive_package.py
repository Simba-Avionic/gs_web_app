import radio_utils

def send_data(serial_conn, data):
    serial_conn.write(data)

def receive_data(serial_conn):
    return serial_conn.read_all()

def main():
    selected_port, detected_baud = radio_utils.pick_pickables()
    try:
        with radio_utils.serial.Serial(selected_port, detected_baud, timeout=1) as ser:
            while True:
                received_data = receive_data(ser)
                if received_data:
                    print(f'Received: {received_data}')
                    # Echo the received data back to the transmitter
                    send_data(ser,received_data)
                    print(f'Echoed back: {received_data}')
                radio_utils.time.sleep(0.1)  # Check for received data every 100ms
    except radio_utils.serial.SerialException as e:
        print(f'Error: {e}')
    except Exception as e:
        print(f'Unexpected error: {e}')

if __name__ == '__main__':
    main()
    