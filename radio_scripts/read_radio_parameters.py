import radio_utils

def main():
    selected_port, detected_baud = radio_utils.pick_pickables()
    radio = radio_utils.RadioModule(selected_port,detected_baud)
    in_command_mode = False
    max_iter = 5
    i = 1
    while (not in_command_mode) and i <= max_iter:
        print("Read attempt no. " + str(i))
        # Enter AT command mode
        in_command_mode = radio.enter_command_mode()
        # Read parameters using ATI5
        radio.display_current_parameters()
        # Exit AT command mode
        i += 1
    radio.leave_command_mode()

if __name__ == '__main__':
    main()