from pymavlink import mavutil
import os
import time

os.environ["MAVLINK_DIALECT"] = "simba"

master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600, dialect="simba")

print("Mavlink connection established. Waiting for heartbeat...")

# master.wait_heartbeat()

print("Heartbeat received. Sending custom message...")

def send_all_messages():
    timestamp = int(time.time() * 1e6)
    
    # SIMBA_MAX_ALTITUDE
    master.mav.simba_max_altitude_send(
        timestamp,
        1000  # alt
    )
    
    # SIMBA_CMD_VALVE_CONTROL
    master.mav.simba_cmd_valve_control_send(
        timestamp,
        1  # cmd_valve_control
    )
    
    # SIMBA_HEARTBEAT_1
    master.mav.simba_heartbeat_1_send(
        timestamp,
        1  # computer_status
    )
    
    # SIMBA_HEARTBEAT_2
    master.mav.simba_heartbeat_2_send(
        timestamp,
        1  # computer_status
    )
    
    # SIMBA_CMD_HOLD
    master.mav.simba_cmd_hold_send(
        timestamp,
        1  # cmd_hold
    )
    
    # SIMBA_CMD_ABORT
    master.mav.simba_cmd_abort_send(
        timestamp,
        1  # cmd_abort
    )
    
    # SIMBA_ACTUATOR
    master.mav.simba_actuator_send(
        timestamp,
        1,  # values
        0   # errors
    )
    
    # SIMBA_TANK_TEMPERATURE_1
    master.mav.simba_tank_temperature_1_send(
        timestamp,
        25,  # temp
        0    # sensor_error
    )
    
    # SIMBA_TANK_TEMPERATURE_2
    master.mav.simba_tank_temperature_2_send(
        timestamp,
        25,  # temp
        0    # sensor_error
    )
    
    # SIMBA_TANK_PRESSURE_1
    master.mav.simba_tank_pressure_1_send(
        timestamp,
        1.0,  # pressure
        0     # error
    )
    
    # SIMBA_TANK_PRESSURE_2
    master.mav.simba_tank_pressure_2_send(
        timestamp,
        1.0,  # pressure
        0     # error
    )
    
    # SIMBA_GPS
    master.mav.simba_gps_send(
        timestamp,
        123456789,  # lat
        987654321,  # lon
        1000        # alt
    )

while True:
    send_all_messages()
    time.sleep(1)
