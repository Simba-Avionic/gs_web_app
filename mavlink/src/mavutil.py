#!/usr/bin/env python3

from __future__ import print_function
from builtins import object

import math, struct, time, os, fnmatch, sys
import select
import copy
import re
from pymavlink import mavexpression

# We want to re-export x25crc here
from pymavlink.generator.mavcrc import x25crc as x25crc

is_py3 = sys.version_info >= (3,0)
supports_type_annotations = sys.version_info >= (3,6)

# maximum packet length for a single receive call - use the UDP limit
UDP_MAX_PACKET_LEN = 65535

# Store the MAVLink library for the currently-selected dialect
# (set by set_dialect())
mavlink = None

# Store the mavlink file currently being operated on
# (set by mavlink_connection())
mavfile_global = None

# If the caller hasn't specified a particular native/legacy version, use this
default_native = False

# link_id used for signing
global_link_id = 0

def mavlink10():
    '''return True if using MAVLink 1.0 or later'''
    return not 'MAVLINK09' in os.environ

def mavlink20():
    '''return True if using MAVLink 2.0'''
    return 'MAVLINK20' in os.environ

def evaluate_expression(expression, vars, nocondition=False):
    '''evaluation an expression'''
    return mavexpression.evaluate_expression(expression, vars, nocondition)

def evaluate_condition(condition, vars):
    '''evaluation a conditional (boolean) statement'''
    if condition is None:
        return True
    v = evaluate_expression(condition, vars)
    if v is None:
        return False
    return v

def u_ord(c):
    if is_py3:
        return c
    return ord(c)

class location(object):
    '''represent a GPS coordinate'''
    def __init__(self, lat, lng, alt=0, heading=0):
        self.lat = lat  # in degrees
        self.lng = lng  # in degrees
        self.alt = alt  # in metres
        self.heading = heading

    def __str__(self):
        return "lat=%.6f,lon=%.6f,alt=%.1f" % (self.lat, self.lng, self.alt)

def add_message(messages, mtype, msg):
    '''add a msg to array of messages, taking account of instance messages'''
    if msg._instance_field is None or getattr(msg, msg._instance_field, None) is None:
        # simple case, no instance field
        messages[mtype] = msg
        return
    instance_value = getattr(msg, msg._instance_field)
    if not mtype in messages:
        messages[mtype] = copy.copy(msg)
        messages[mtype]._instances = {}
        messages[mtype]._instances[instance_value] = msg
        messages["%s[%s]" % (mtype, str(instance_value))] = copy.copy(msg)
        return
    messages[mtype]._instances[instance_value] = msg
    prev_instances = messages[mtype]._instances
    messages[mtype] = copy.copy(msg)
    messages[mtype]._instances = prev_instances
    messages["%s[%s]" % (mtype, str(instance_value))] = copy.copy(msg)

def set_dialect(dialect, with_type_annotations=None):
    '''set the MAVLink dialect to work with.
    For example, set_dialect("ardupilotmega")
    '''
    global mavlink, current_dialect
    # from .generator import mavparse

    if with_type_annotations is None:
        with_type_annotations = supports_type_annotations

    legacy_python_module = "python2." if not with_type_annotations else ""
    if 'MAVLINK20' in os.environ:
        # wire_protocol = mavparse.PROTOCOL_2_0
        modname = "pymavlink.dialects.v20." + legacy_python_module + dialect
    elif mavlink is None or mavlink.WIRE_PROTOCOL_VERSION == "1.0" or not 'MAVLINK09' in os.environ:
        # wire_protocol = mavparse.PROTOCOL_1_0
        modname = "pymavlink.dialects.v10." + legacy_python_module + dialect
    else:
        # wire_protocol = mavparse.PROTOCOL_0_9
        modname = "pymavlink.dialects.v09." + legacy_python_module + dialect

    try:
        mod = __import__(modname)
    except Exception:
        # auto-generate the dialect module
        # from .generator.mavgen import mavgen_python_dialect
        # mavgen_python_dialect(dialect, wire_protocol, with_type_annotations=with_type_annotations)
        # mod = __import__(modname)
        pass

    components = modname.split('.')
    for comp in components[1:]:
        mod = getattr(mod, comp)
    current_dialect = dialect
    mavlink = mod

# Set the default dialect. This is done here as it needs to be after the function declaration
set_dialect(os.environ['MAVLINK_DIALECT'])

class mavfile_state(object):
    '''state for a particular system id'''
    def __init__(self):
        self.messages = { 'MAV' : self }
        self.flightmode = "UNKNOWN"
        self.vehicle_type = "UNKNOWN"
        self.mav_type = None
        self.mav_autopilot = None
        self.base_mode = 0
        self.armed = False # canonical arm state for the vehicle as a whole

class param_state(object):
    '''state for a particular system id/component id pair'''
    def __init__(self):
        self.params = {}

class mavfile(object):
    '''a generic mavlink port'''
    def __init__(self, fd, address, source_system=255, source_component=0, notimestamps=False, input=True, use_native=default_native):
        global mavfile_global
        if input:
            mavfile_global = self
        self.fd = fd
        self.sysid = 0
        self.param_sysid = (0,0)
        self.address = address
        self.timestamp = 0
        self.last_seq = {}
        self.mav_loss = 0
        self.mav_count = 0
        self.param_fetch_start = 0

        # state for each sysid
        self.sysid_state = {}
        self.sysid_state[self.sysid] = mavfile_state()

        # param state for each sysid/compid tuple
        self.param_state = {}
        self.param_state[self.param_sysid] = param_state()
        
        # status of param fetch, indexed by sysid,compid tuple
        self.source_system = source_system
        self.source_component = source_component
        self.first_byte = True
        self.robust_parsing = True
        self.mav = mavlink.MAVLink(self, srcSystem=self.source_system, srcComponent=self.source_component, use_native=use_native)
        self.mav.robust_parsing = self.robust_parsing
        self.logfile = None
        self.logfile_raw = None
        self.start_time = time.time()
        self.message_hooks = []
        self.idle_hooks = []
        self.uptime = 0.0
        self.notimestamps = notimestamps
        self._timestamp = None
        self.WIRE_PROTOCOL_VERSION = mavlink.WIRE_PROTOCOL_VERSION
        self.stop_on_EOF = False
        self.portdead = False

    @property
    def target_system(self):
        return self.sysid

    @property
    def target_component(self):
        return self.param_sysid[1]
    
    @target_system.setter
    def target_system(self, value):
        self.sysid = value
        if not self.sysid in self.sysid_state:
            self.sysid_state[self.sysid] = mavfile_state()
        if self.sysid != self.param_sysid[0]:
            self.param_sysid = (self.sysid, self.param_sysid[1])
            if not self.param_sysid in self.param_state:
                self.param_state[self.param_sysid] = param_state()

    @target_component.setter
    def target_component(self, value):
        if value != self.param_sysid[1]:
            self.param_sysid = (self.param_sysid[0], value)
            if not self.param_sysid in self.param_state:
                self.param_state[self.param_sysid] = param_state()

    @property
    def params(self):
        if self.param_sysid[1] == 0:
            eff_tuple = (self.sysid, 1)
            if eff_tuple in self.param_state:
                return getattr(self.param_state[eff_tuple],'params')
        return getattr(self.param_state[self.param_sysid],'params')

    @property
    def messages(self):
        return getattr(self.sysid_state[self.sysid],'messages')

    @property
    def flightmode(self):
        return getattr(self.sysid_state[self.sysid],'flightmode')

    @flightmode.setter
    def flightmode(self, value):
        setattr(self.sysid_state[self.sysid],'flightmode',value)

    @property
    def vehicle_type(self):
        return getattr(self.sysid_state[self.sysid],'vehicle_type')

    @vehicle_type.setter
    def vehicle_type(self, value):
        setattr(self.sysid_state[self.sysid],'vehicle_type',value)

    @property
    def mav_type(self):
        return getattr(self.sysid_state[self.sysid],'mav_type')

    @mav_type.setter
    def mav_type(self, value):
        setattr(self.sysid_state[self.sysid],'mav_type',value)
    
    @property
    def base_mode(self):
        return getattr(self.sysid_state[self.sysid],'base_mode')

    @base_mode.setter
    def base_mode(self, value):
        setattr(self.sysid_state[self.sysid],'base_mode',value)
    
    def auto_mavlink_version(self, buf):
        '''auto-switch mavlink protocol version'''
        global mavlink
        if len(buf) == 0:
            return
        try:
            magic = ord(buf[0])
        except:
            magic = buf[0]
        if not magic in [ 85, 254, 253 ]:
            return
        self.first_byte = False
        if self.WIRE_PROTOCOL_VERSION == "0.9" and magic == 254:
            self.WIRE_PROTOCOL_VERSION = "1.0"
            set_dialect(current_dialect)
        elif self.WIRE_PROTOCOL_VERSION == "1.0" and magic == 85:
            self.WIRE_PROTOCOL_VERSION = "0.9"
            os.environ['MAVLINK09'] = '1'
            set_dialect(current_dialect)
        elif self.WIRE_PROTOCOL_VERSION != "2.0" and magic == 253:
            self.WIRE_PROTOCOL_VERSION = "2.0"
            os.environ['MAVLINK20'] = '1'
            set_dialect(current_dialect)
        else:
            return
        # switch protocol 
        (callback, callback_args, callback_kwargs) = (self.mav.callback,
                                                      self.mav.callback_args,
                                                      self.mav.callback_kwargs)
        self.mav = mavlink.MAVLink(self, srcSystem=self.source_system, srcComponent=self.source_component)
        self.mav.robust_parsing = self.robust_parsing
        self.WIRE_PROTOCOL_VERSION = mavlink.WIRE_PROTOCOL_VERSION
        (self.mav.callback, self.mav.callback_args, self.mav.callback_kwargs) = (callback,
                                                                                 callback_args,
                                                                                 callback_kwargs)

    def recv(self, n=None):
        '''default recv method'''
        raise RuntimeError('no recv() method supplied')

    def close(self, n=None):
        '''default close method'''
        raise RuntimeError('no close() method supplied')

    def write(self, buf):
        '''default write method'''
        raise RuntimeError('no write() method supplied')


    def select(self, timeout):
        '''wait for up to timeout seconds for more data'''
        if self.fd is None:
            time.sleep(min(timeout,0.5))
            return True
        try:
            (rin, win, xin) = select.select([self.fd], [], [], timeout)
        except select.error:
            return False
        return len(rin) == 1

    def pre_message(self):
        '''default pre message call'''
        return

    def set_rtscts(self, enable):
        '''enable/disable RTS/CTS if applicable'''
        return

    def probably_vehicle_heartbeat(self, msg):
        if msg.get_srcComponent() == mavlink.MAV_COMP_ID_GIMBAL:
            return False
        if msg.type in (mavlink.MAV_TYPE_GCS,
                        mavlink.MAV_TYPE_GIMBAL,
                        mavlink.MAV_TYPE_ADSB,
                        mavlink.MAV_TYPE_ONBOARD_CONTROLLER):
            return False
        if msg.autopilot in frozenset([
                mavlink.MAV_AUTOPILOT_INVALID
                ]):
            return False
        return True

    def post_message(self, msg):
        '''default post message call'''
        if '_posted' in msg.__dict__:
            return
        msg._posted = True
        msg._timestamp = time.time()
        type = msg.get_type()

        if 'usec' in msg.__dict__:
            self.uptime = msg.usec * 1.0e-6
        if 'time_boot_ms' in msg.__dict__:
            self.uptime = msg.time_boot_ms * 1.0e-3

        if self._timestamp is not None:
            if self.notimestamps:
                msg._timestamp = self.uptime
            else:
                msg._timestamp = self._timestamp

        src_system = msg.get_srcSystem()
        src_component = msg.get_srcComponent()
        src_tuple = (src_system, src_component)

        radio_tuple = (ord('3'), ord('D'))

        if not src_system in self.sysid_state:
            # we've seen a new system
            self.sysid_state[src_system] = mavfile_state()

        add_message(self.sysid_state[src_system].messages, type, msg)

        if src_tuple == radio_tuple:
            # as a special case radio msgs are added for all sysids
            for s in self.sysid_state.keys():
                self.sysid_state[s].messages[type] = msg

        if not (src_tuple == radio_tuple or msg.get_msgId() < 0):
            # Don't use unknown messages to calculate number of lost packets
            if not src_tuple in self.last_seq:
                last_seq = -1
            else:
                last_seq = self.last_seq[src_tuple]
            seq = (last_seq+1) % 256
            seq2 = msg.get_seq()
            if seq != seq2 and last_seq != -1:
                diff = (seq2 - seq) % 256
                self.mav_loss += diff
                #print("lost %u seq=%u seq2=%u last_seq=%u src_tupe=%s %s" % (diff, seq, seq2, last_seq, str(src_tuple), msg.get_type()))
            self.last_seq[src_tuple] = seq2
            self.mav_count += 1
        
        self.timestamp = msg._timestamp
        if type == 'HEARTBEAT' and self.probably_vehicle_heartbeat(msg):
            if self.sysid == 0:
                # lock onto id tuple of first vehicle heartbeat
                self.sysid = src_system
            if float(mavlink.WIRE_PROTOCOL_VERSION) >= 1:
                # self.flightmode = mode_string_v10(msg)
                self.mav_type = msg.type
                self.base_mode = msg.base_mode
                self.sysid_state[src_system].armed = (msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.sysid_state[src_system].mav_type = msg.type
                self.sysid_state[src_system].mav_autopilot = msg.autopilot
        elif type == 'HIGH_LATENCY2':
            if self.sysid == 0:
                # lock onto id tuple of first vehicle heartbeat
                self.sysid = src_system
            # self.flightmode = mode_string_v10(msg)
            self.mav_type = msg.type
            if msg.autopilot == mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                self.base_mode = msg.custom0
                self.sysid_state[src_system].armed = (msg.custom0 & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.sysid_state[src_system].mav_type = msg.type
            self.sysid_state[src_system].mav_autopilot = msg.autopilot

        elif type == 'PARAM_VALUE':
            if not src_tuple in self.param_state:
                self.param_state[src_tuple] = param_state()
            self.param_state[src_tuple].params[msg.param_id] = msg.param_value
        elif type == 'SYS_STATUS' and mavlink.WIRE_PROTOCOL_VERSION == '0.9':
            self.flightmode = None
        elif type == 'GPS_RAW':
            if self.sysid_state[src_system].messages['HOME'].fix_type < 2:
                self.sysid_state[src_system].messages['HOME'] = msg
        elif type == 'GPS_RAW_INT':
            if self.sysid_state[src_system].messages['HOME'].fix_type < 3:
                self.sysid_state[src_system].messages['HOME'] = msg
        for hook in self.message_hooks:
            hook(self, msg)

        if (msg.get_signed() and
            self.mav.signing.link_id == 0 and
            msg.get_link_id() != 0 and
            self.target_system == msg.get_srcSystem() and
            self.target_component == msg.get_srcComponent()):
            # change to link_id from incoming packet
            self.mav.signing.link_id = msg.get_link_id()


    def packet_loss(self):
        '''packet loss as a percentage'''
        if self.mav_count == 0:
            return 0
        return (100.0*self.mav_loss)/(self.mav_count+self.mav_loss)


    def recv_msg(self):
        '''message receive routine'''
        self.pre_message()
        while True:
            n = self.mav.bytes_needed()
            s = self.recv(n)
            numnew = len(s)

            if numnew != 0:
                if self.logfile_raw:
                    if is_py3:
                        self.logfile_raw.write(s)
                    else:
                        self.logfile_raw.write(str(s))
                if self.first_byte:
                    self.auto_mavlink_version(s)

            # We always call parse_char even if the new string is empty, because the existing message buf might already have some valid packet
            # we can extract
            msg = self.mav.parse_char(s)
            if msg:
                if self.logfile and  msg.get_type() != 'BAD_DATA' :
                    usec = int(time.time() * 1.0e6) & ~3
                    if is_py3:
                        self.logfile.write(struct.pack('>Q', usec) + msg.get_msgbuf())
                    else:
                        self.logfile.write(str(struct.pack('>Q', usec) + msg.get_msgbuf()))
                self.post_message(msg)
                return msg
            else:
                # if we failed to parse any messages _and_ no new bytes arrived, return immediately so the client has the option to
                # timeout
                if numnew == 0:
                    return None
                
    def recv_match(self, condition=None, type=None, blocking=False, timeout=None):
        '''recv the next MAVLink message that matches the given condition
        type can be a string or a list of strings'''
        if type is not None and not isinstance(type, list) and not isinstance(type, set):
            type = [type]
        start_time = time.time()
        while True:
            if timeout is not None:
                now = time.time()
                if now < start_time:
                    start_time = now # If an external process rolls back system time, we should not spin forever.
                if start_time + timeout < time.time():
                    return None
            m = self.recv_msg()
            if m is None:
                if blocking:
                    for hook in self.idle_hooks:
                        hook(self)
                    if timeout is None:
                        self.select(0.05)
                    else:
                        self.select(timeout/2)
                    continue
                return None
            if type is not None and not m.get_type() in type:
                continue
            if not evaluate_condition(condition, self.messages):
                continue
            return m

    def check_condition(self, condition):
        '''check if a condition is true'''
        return evaluate_condition(condition, self.messages)

    def mavlink10(self):
        '''return True if using MAVLink 1.0 or later'''
        return float(self.WIRE_PROTOCOL_VERSION) >= 1

    def mavlink20(self):
        '''return True if using MAVLink 2.0 or later'''
        return float(self.WIRE_PROTOCOL_VERSION) >= 2

    def setup_logfile(self, logfile, mode='wb'):
        '''start logging to the given logfile, with timestamps'''
        self.logfile = open(logfile, mode=mode)

    def setup_logfile_raw(self, logfile, mode='wb'):
        '''start logging raw bytes to the given logfile, without timestamps'''
        self.logfile_raw = open(logfile, mode=mode)

    def wait_heartbeat(self, blocking=True, timeout=None):
        '''wait for a heartbeat so we know the target system IDs'''
        return self.recv_match(type='HEARTBEAT', blocking=blocking, timeout=timeout)

    def param_fetch_all(self):
        '''initiate fetch of all parameters'''
        if time.time() - self.param_fetch_start < 2.0:
            # don't fetch too often
            return
        self.param_fetch_start = time.time()
        self.mav.param_request_list_send(self.target_system, self.target_component)

    def param_fetch_one(self, name):
        '''initiate fetch of one parameter'''
        try:
            idx = int(name)
            self.mav.param_request_read_send(self.target_system, self.target_component, b"", idx)
        except Exception:
            if sys.version_info.major >= 3 and not isinstance(name, bytes):
                name = bytes(name,'ascii')
            self.mav.param_request_read_send(self.target_system, self.target_component, name, -1)

    def time_since(self, mtype):
        '''return the time since the last message of type mtype was received'''
        if not mtype in self.messages:
            return time.time() - self.start_time
        return time.time() - self.messages[mtype]._timestamp

    def param_set_send(self, parm_name, parm_value, parm_type=None):
        '''wrapper for parameter set'''
        if self.mavlink10():
            if parm_type is None:
                parm_type = mavlink.MAVLINK_TYPE_FLOAT
            self.mav.param_set_send(self.target_system, self.target_component,
                                    parm_name.encode('utf8'), parm_value, parm_type)
        else:
            self.mav.param_set_send(self.target_system, self.target_component,
                                    parm_name.encode('utf8'), parm_value)

    def set_relay(self, relay_pin=0, state=True):
        '''Set relay_pin to value of state'''
        if self.mavlink10():
            self.mav.command_long_send(
                self.target_system,  # target_system
                self.target_component, # target_component
                mavlink.MAV_CMD_DO_SET_RELAY, # command
                0, # Confirmation
                relay_pin, # Relay Number
                int(state), # state (1 to indicate arm)
                0, # param3 (all other params meaningless)
                0, # param4
                0, # param5
                0, # param6
                0) # param7
        else:
            print("Setting relays not supported.")

    def calibrate_level(self):
        '''calibrate accels (1D version)'''
        self.mav.command_long_send(self.target_system, self.target_component,
                                   mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                                   1, 1, 0, 0, 0, 0, 0)

    def calibrate_pressure(self):
        '''calibrate pressure'''
        if self.mavlink10():
            self.mav.command_long_send(self.target_system, self.target_component,
                                       mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                                       0, 0, 1, 0, 0, 0, 0)
        else:
            MAV_ACTION_CALIBRATE_PRESSURE = 20
            self.mav.action_send(self.target_system, self.target_component, MAV_ACTION_CALIBRATE_PRESSURE)

    def reboot_autopilot(self, hold_in_bootloader=False, force=False):
        '''reboot the autopilot'''
        if self.mavlink10():
            if hold_in_bootloader:
                param1 = 3
            else:
                param1 = 1
            if force:
                param6 = 20190226
            else:
                param6 = 0
            self.mav.command_long_send(self.target_system, self.target_component,
                                       mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
                                       param1, 0, 0, 0, 0, param6, 0)

    def wait_gps_fix(self):
        self.recv_match(type='VFR_HUD', blocking=True)
        if self.mavlink10():
            self.recv_match(type='GPS_RAW_INT', blocking=True,
                            condition='GPS_RAW_INT.fix_type>=3 and GPS_RAW_INT.lat != 0')
        else:
            self.recv_match(type='GPS_RAW', blocking=True,
                            condition='GPS_RAW.fix_type>=2 and GPS_RAW.lat != 0')

    def location(self, relative_alt=False):
        '''return current location'''
        self.wait_gps_fix()
        # wait for another VFR_HUD, to ensure we have correct altitude
        self.recv_match(type='VFR_HUD', blocking=True)
        self.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if relative_alt:
            alt = self.messages['GLOBAL_POSITION_INT'].relative_alt*0.001
        else:
            alt = self.messages['VFR_HUD'].alt
        return location(self.messages['GPS_RAW_INT'].lat*1.0e-7,
                        self.messages['GPS_RAW_INT'].lon*1.0e-7,
                        alt,
                        self.messages['VFR_HUD'].heading)

    def arducopter_arm(self):
        '''arm motors (arducopter only)'''
        if self.mavlink10():
            self.mav.command_long_send(
                self.target_system,  # target_system
                self.target_component,
                mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # command
                0, # confirmation
                1, # param1 (1 to indicate arm)
                0, # param2 (all other params meaningless)
                0, # param3
                0, # param4
                0, # param5
                0, # param6
                0) # param7

    def arducopter_disarm(self):
        '''disarm motors (arducopter only)'''
        if self.mavlink10():
            self.mav.command_long_send(
                self.target_system,  # target_system
                self.target_component,
                mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # command
                0, # confirmation
                0, # param1 (0 to indicate disarm)
                0, # param2 (all other params meaningless)
                0, # param3
                0, # param4
                0, # param5
                0, # param6
                0) # param7

    def motors_armed(self):
        '''return true if motors armed'''
        return self.sysid_state[self.sysid].armed

    def motors_armed_wait(self):
        '''wait for motors to be armed'''
        while True:
            m = self.wait_heartbeat()
            if self.motors_armed():
                return

    def motors_disarmed_wait(self):
        '''wait for motors to be disarmed'''
        while True:
            m = self.wait_heartbeat()
            if not self.motors_armed():
                return


    def field(self, type, field, default=None):
        '''convenient function for returning an arbitrary MAVLink
           field with a default'''
        if not type in self.messages:
            return default
        return getattr(self.messages[type], field, default)

    def param(self, name, default=None):
        '''convenient function for returning an arbitrary MAVLink
           parameter with a default'''
        if not name in self.params:
            return default
        return self.params[name]

    def setup_signing(self, secret_key, sign_outgoing=True, allow_unsigned_callback=None, initial_timestamp=None, link_id=None):
        '''setup for MAVLink2 signing'''
        self.mav.signing.secret_key = secret_key
        self.mav.signing.sign_outgoing = sign_outgoing
        self.mav.signing.allow_unsigned_callback = allow_unsigned_callback
        if link_id is None:
            # auto-increment the link_id for each link
            global global_link_id
            link_id = global_link_id
            global_link_id = min(global_link_id + 1, 255)
        self.mav.signing.link_id = link_id
        if initial_timestamp is None:
            # timestamp is time since 1/1/2015
            epoch_offset = 1420070400
            now = max(time.time(), epoch_offset)
            initial_timestamp = now - epoch_offset
            initial_timestamp = int(initial_timestamp * 100 * 1000)
        # initial_timestamp is in 10usec units
        self.mav.signing.timestamp = initial_timestamp

    def disable_signing(self):
        '''disable MAVLink2 signing'''
        self.mav.signing.secret_key = None
        self.mav.signing.sign_outgoing = False
        self.mav.signing.allow_unsigned_callback = None
        self.mav.signing.link_id = 0
        self.mav.signing.timestamp = 0

def set_close_on_exec(fd):
    '''set the close on exec flag on a file descriptor. Ignore exceptions'''
    try:
        import fcntl
        flags = fcntl.fcntl(fd, fcntl.F_GETFD)
        flags |= fcntl.FD_CLOEXEC
        fcntl.fcntl(fd, fcntl.F_SETFD, flags)
    except Exception:
        pass

class FakeSerial():
    def __init__(self):
        pass
    def read(self, len):
        return ""
    def write(self, buf):
        raise Exception("write always fails")
    def inWaiting(self):
        return 0
    def close(self):
        pass

class mavserial(mavfile):
    '''a serial mavlink port'''
    def __init__(self, device, baud=115200, autoreconnect=False, source_system=255, source_component=0, use_native=default_native, force_connected=False):
        import serial
        if ',' in device and not os.path.exists(device):
            device, baud = device.split(',')
        self.baud = baud
        self.device = device
        self.autoreconnect = autoreconnect
        self.force_connected = force_connected
        # we rather strangely set the baudrate initially to 1200, then change to the desired
        # baudrate. This works around a kernel bug on some Linux kernels where the baudrate
        # is not set correctly
        try:
            self.port = serial.Serial(self.device, 1200, timeout=0,
                                      dsrdtr=False, rtscts=False, xonxoff=False)
        except serial.SerialException as e:
            if not force_connected:
                raise e
            self.port = FakeSerial()

        try:
            fd = self.port.fileno()
            set_close_on_exec(fd)
        except Exception:
            fd = None
        self.set_baudrate(self.baud)
        mavfile.__init__(self, fd, device, source_system=source_system, source_component=source_component, use_native=use_native)
        self.rtscts = False

    def set_rtscts(self, enable):
        '''enable/disable RTS/CTS if applicable'''
        try:
            self.port.setRtsCts(enable)
        except Exception:
            self.port.rtscts = enable
        self.rtscts = enable

    def set_baudrate(self, baudrate):
        '''set baudrate'''
        try:
            self.port.setBaudrate(baudrate)
        except Exception:
            # for pySerial 3.0, which doesn't have setBaudrate()
            self.port.baudrate = baudrate
    
    def close(self):
        self.port.close()

    def recv(self,n=None):
        if n is None:
            n = self.mav.bytes_needed()
        if self.fd is None:
            waiting = self.port.inWaiting()
            if waiting < n:
                n = waiting
        ret = self.port.read(n)
        return ret

    def write(self, buf):
        try:
            return self.port.write(bytes(buf))
        except Exception:
            if not self.portdead:
                print("Device %s is dead" % self.device)
            self.portdead = True
            if self.autoreconnect:
                self.reset()
            return -1
            
    def reset(self):
        import serial
        try:
            try:
                newport = serial.Serial(self.device, self.baud, timeout=0,
                                        dsrdtr=False, rtscts=False, xonxoff=False)
            except serial.SerialException as e:
                if not self.force_connected:
                    raise e
                newport = FakeSerial()
                return False
            self.port.close()
            self.port = newport
            print("Device %s reopened OK" % self.device)
            self.portdead = False
            try:
                self.fd = self.port.fileno()
            except Exception:
                self.fd = None
            self.set_baudrate(self.baud)
            if self.rtscts:
                self.set_rtscts(self.rtscts)
            return True
        except Exception:
            return False

def mavlink_connection(device, baud=115200, source_system=255, source_component=0,
                       planner_format=None, write=False, append=False,
                       robust_parsing=True, notimestamps=False, input=True,
                       dialect=None, autoreconnect=False, zero_time_base=False,
                       retries=3, use_native=default_native,
                       force_connected=False, progress_callback=None,
                       udp_timeout=0, **opts):
    '''open a serial, UDP, TCP or file mavlink connection'''
    global mavfile_global

    if force_connected:
        # force_connected implies autoreconnect
        autoreconnect = True

    if dialect is not None:
        set_dialect(dialect)

    if device.lower().endswith('.bin') or device.lower().endswith('.px4log'):
        # support dataflash logs
        from pymavlink import DFReader
        m = DFReader.DFReader_binary(device, zero_time_base=zero_time_base, progress_callback=progress_callback)
        mavfile_global = m
        return m

    if device.lower().startswith('csv:'):
        # support CSV logs
        from pymavlink import CSVReader
        # special-case for users wanting a : separator:
        colon_separator_re = ""
        if re.match(".*separator=::?.*", device):
            opts["separator"] = ":"
            device = re.sub(":separator=:", "", device)
        components = device.split(":")
        filename = components[1]
        for nv in components[2:]:
            (name, value) = nv.split('=')
            opts[name] = value
        m = CSVReader.CSVReader(filename,
                                zero_time_base=zero_time_base,
                                progress_callback=progress_callback,
                                **opts)
        mavfile_global = m
        return m

    if device.endswith('.log'):
        # support dataflash text logs
        from pymavlink import DFReader
        if DFReader.DFReader_is_text_log(device):
            m = DFReader.DFReader_text(device, zero_time_base=zero_time_base, progress_callback=progress_callback)
            mavfile_global = m
            return m    

    return mavserial(device,
                     baud=baud,
                     source_system=source_system,
                     source_component=source_component,
                     autoreconnect=autoreconnect,
                     use_native=use_native,
                     force_connected=force_connected)

class periodic_event(object):
    '''a class for fixed frequency events'''
    def __init__(self, frequency):
        self.frequency = float(frequency)
        self.last_time = time.time()

    def force(self):
        '''force immediate triggering'''
        self.last_time = 0
        
    def trigger(self):
        '''return True if we should trigger now'''
        tnow = time.time()

        if tnow < self.last_time:
            print("Warning, time moved backwards. Restarting timer.")
            self.last_time = tnow

        if self.last_time + (1.0/self.frequency) <= tnow:
            self.last_time = tnow
            return True
        return False

try:
    from curses import ascii
    have_ascii = True
except:
    have_ascii = False

def is_printable(c):
    '''see if a character is printable'''
    global have_ascii
    if have_ascii:
        return ascii.isprint(c)
    if isinstance(c, int):
        ic = c
    else:
        ic = ord(c)
    return ic >= 32 and ic <= 126

def all_printable(buf):
    '''see if a string is all printable'''
    for c in buf:
        if not is_printable(c) and not c in ['\r', '\n', '\t'] and not c in [ord('\r'), ord('\n'), ord('\t')]:
            return False
    return True

class SerialPort(object):
    '''auto-detected serial port'''
    def __init__(self, device, description=None, hwid=None):
        self.device = device
        self.description = description
        self.hwid = hwid

    def __str__(self):
        ret = self.device
        if self.description is not None:
            ret += " : " + self.description
        if self.hwid is not None:
            ret += " : " + self.hwid
        return ret

def auto_detect_serial_win32(preferred_list=['*']):
    '''try to auto-detect serial ports on win32'''
    try:
        from serial.tools.list_ports_windows import comports
        list = sorted(comports())
    except:
        return []
    ret = []
    others = []
    for port, description, hwid in list:
        matches = False
        p = SerialPort(port, description=description, hwid=hwid)
        for preferred in preferred_list:
            if fnmatch.fnmatch(description, preferred) or fnmatch.fnmatch(hwid, preferred):
                matches = True
        if matches:
            ret.append(p)
        else:
            others.append(p)
    if len(ret) > 0:
        return ret
    # now the rest
    ret.extend(others)
    return ret
               

def auto_detect_serial_unix(preferred_list=['*']):
    '''try to auto-detect serial ports on unix'''
    import glob
    glist = glob.glob('/dev/ttyS*') + glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') + glob.glob('/dev/serial/by-id/*')
    ret = []
    others = []
    # try preferred ones first
    for d in glist:
        matches = False
        for preferred in preferred_list:
            if fnmatch.fnmatch(d, preferred):
                matches = True
        if matches:
            ret.append(SerialPort(d))
        else:
            others.append(SerialPort(d))
    if len(ret) > 0:
        return ret
    ret.extend(others)
    return ret

def auto_detect_serial(preferred_list=['*']):
    '''try to auto-detect serial port'''
    # see if 
    if os.name == 'nt':
        return auto_detect_serial_win32(preferred_list=preferred_list)
    return auto_detect_serial_unix(preferred_list=preferred_list)

class MavlinkSerialPort(object):
        '''an object that looks like a serial port, but
        transmits using mavlink SERIAL_CONTROL packets'''
        def __init__(self, portname, baudrate, devnum=0, devbaud=0, timeout=3, debug=0):
                from . import mavutil

                self.baudrate = 0
                self.timeout = timeout
                self._debug = debug
                self.buf = bytearray()
                self.port = devnum
                self.debug("Connecting with MAVLink to %s ..." % portname)
                self.mav = mavutil.mavlink_connection(portname, autoreconnect=True, baud=baudrate)
                self.mav.wait_heartbeat()
                self.debug("HEARTBEAT OK\n")
                if devbaud != 0:
                    self.setBaudrate(devbaud)
                self.debug("Locked serial device\n")

        def debug(self, s, level=1):
                '''write some debug text'''
                if self._debug >= level:
                        print(s)

        def write(self, b):
                '''write some bytes'''
                from . import mavutil
                while len(b) > 0:
                        n = len(b)
                        if n > 70:
                                n = 70
                        buf = bytearray(b[:])
                        buf.extend([0]*(70-len(buf)))
                        self.mav.mav.serial_control_send(self.port,
                                                         mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                                         mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                                                         0,
                                                         0,
                                                         n,
                                                         buf)
                        b = b[n:]

        def _recv(self):
                '''read some bytes into self.buf'''
                from . import mavutil
                start_time = time.time()
                while time.time() < start_time + self.timeout:
                        m = self.mav.recv_match(condition='SERIAL_CONTROL.count!=0',
                                                type='SERIAL_CONTROL', blocking=False, timeout=0)
                        if m is not None and m.count != 0:
                                break
                        self.mav.mav.serial_control_send(self.port,
                                                         mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                                         mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                                                         0,
                                                         0,
                                                         0, [0]*70)
                        m = self.mav.recv_match(condition='SERIAL_CONTROL.count!=0',
                                                type='SERIAL_CONTROL', blocking=True, timeout=0.01)
                        if m is not None and m.count != 0:
                                break
                if m is not None:
                        if self._debug > 2:
                                print(m)
                        data = m.data[:m.count]
                        self.buf.extend(data)

        def read(self, n):
                '''read some bytes'''
                if len(self.buf) == 0:
                        self._recv()
                if len(self.buf) > 0:
                        if n > len(self.buf):
                                n = len(self.buf)
                        ret = self.buf[:n]
                        self.buf = self.buf[n:]
                        return ret
                return bytearray()

        def flushInput(self):
                '''flush any pending input'''
                self.buf = bytearray()
                saved_timeout = self.timeout
                self.timeout = 0.5
                self._recv()
                self.timeout = saved_timeout
                self.buf = bytearray()
                self.debug("flushInput")

        def setBaudrate(self, baudrate):
                '''set baudrate'''
                from . import mavutil
                if self.baudrate == baudrate:
                        return
                self.baudrate = baudrate
                self.mav.mav.serial_control_send(self.port,
                                                 mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE,
                                                 0,
                                                 self.baudrate,
                                                 0, [0]*70)
                self.flushInput()
                self.debug("Changed baudrate %u" % self.baudrate)

def decode_bitmask(messagetype, field, value):
    try:
        _class = eval("mavlink.MAVLink_%s_message" % messagetype.lower())
    except AttributeError as e:
        raise AttributeError("No such message type")

    try:
        display = _class.fielddisplays_by_name[field]
    except KeyError as e:
        raise AttributeError("Not a bitmask field (none specified)")

    if display != "bitmask":
        raise ValueError("Not a bitmask field")

    try:
        enum_name = _class.fieldenums_by_name[field]
    except KeyError as e:
        raise AttributeError("No enum found for bitmask field")

    try:
        enum = mavlink.enums[enum_name]
    except KeyError as e:
        raise AttributeError("Did not find specified enumeration (%s)" % enum_name)

    class EnumBitInfo(object):
        def __init__(self, offset, value, name):
            self.offset = offset
            self.value = value
            self.name = name

    ret = []
    for i in range(0, 64):
        bit_value = (1 << i)
        try:
            enum_entry = enum[bit_value]
            enum_entry_name = enum_entry.name
        except KeyError as e:
            enum_entry_name = None
            if value == 0:
                continue
        if value & bit_value:
            ret.append( EnumBitInfo(i, True, enum_entry_name) )
            value = value & ~bit_value
        else:
            ret.append( EnumBitInfo(i, False, enum_entry_name) )
    return ret

'''lookup table to map from a raw unit into a divisor and output unit'''
dump_message_unit_decoder = {
    "d%":     [10.0,       "%"],
    "c%":     [100.0,      "%"],
    "cA":     [100.0,      "A"],
    "cdegC":  [100.0,      "degC"],
    "cdeg":   [100.0,      "deg"],
    "degE5":  [100000.0,   "deg"],
    "degE7":  [10000000.0, "deg"],
    "mG":     [1000.0,     "G"],
    "mrad/s": [1000.0,     "rad/s"],
    "mV":     [1000.0,     "V"]
}

def dump_message_verbose(f, m):
    '''write an excruciatingly detailed dump of message m to file descriptor f'''
    try:
        # __getattr__ may be overridden on m, thus this try/except
        timestamp = m._timestamp
    except AttributeError as e:
        timestamp = ""
    if timestamp != "":
        timestamp = "%s.%02u: " % (
            time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(timestamp)),
            int(timestamp*100.0)%100)
    f.write("%s%s (id=%u) (link=%s) (signed=%s) (seq=%u) (src=%u/%u)\n" % (timestamp, m.get_type(), m.get_msgId(), str(m.get_link_id()), str(m.get_signed()), m.get_seq(), m.get_srcSystem(), m.get_srcComponent()))
    for fieldname in m.get_fieldnames():

        # format in those most boring way possible:
        value = m.format_attr(fieldname)

        # try to add units:
        try:
            units = m.fieldunits_by_name[fieldname]

            # perform simple unit conversions:
            if units in dump_message_unit_decoder:
                divisor = dump_message_unit_decoder[units][0]
                units = dump_message_unit_decoder[units][1]
                if type(value) == list:
                    value = [x/divisor for x in value]
                else:
                    value = value / divisor

            # append degrees to fields in radians:
            if units == "rad":
                value = "%s%s (%sdeg)" % (value, units, math.degrees(value))
            elif units == "rad/s":
                value = "%s%s (%sdeg/s)" % (value, units, math.degrees(value))
            elif units == "rad/s/s":
                value = "%s%s (%sdeg/s/s)" % (value, units, math.degrees(value))

            # append local time if us represents unix time:
            elif units == "us":
                if value > (1<<50):
                    local_time = time.localtime(int(value/1000000))
                    time_str = time.strftime("%Y-%m-%d %H:%M:%S", local_time)
                    tm_zone = local_time.tm_zone
                    value = "%s%s (%s.%06d %s)" % (value, units, time_str, value%1000000, tm_zone)
                elif value > 1000000:
                    value = "%s%s (%ss)" % (value, units, value / 1000000.0)
                else:
                    value = "%s%s" % (value, units)

            # by default, just append the unit:
            else:
                value = "%s%s" % (value, units)
        except AttributeError as e:
            # e.g. BAD_DATA
            pass
        except KeyError as e:
            pass

        # format any bitmask enumerations:
        try:
            display = m.fielddisplays_by_name[fieldname] if fieldname in m.fielddisplays_by_name else ""
            if display == "bitmask":
                # Display bitmasks as hex (dec), regardless of whether there is an enum
                f.write("    %s: 0x%x (%d)\n" % (fieldname, value, value))
                enum_name = m.fieldenums_by_name[fieldname] if fieldname in m.fieldenums_by_name else None
                if enum_name is not None:
                    bits = decode_bitmask(m.get_type(), fieldname, value)
                    for bit in bits:
                        value = bit.value
                        name = bit.name
                        svalue = " "
                        if not value:
                            svalue = "!"
                        if name is None:
                            name = "[UNKNOWN]"
                        f.write("      %s %s\n" % (svalue, name))
                continue
#            except NameError as e:
#                pass
        except AttributeError as e:
            # e.g. BAD_DATA
            pass
        except KeyError as e:
            pass

        # add any enumeration name:
        try:
            enum_name = m.fieldenums_by_name[fieldname]
            try:
                enum_value = mavlink.enums[enum_name][value].name
                value = "%s (%s)" % (value, enum_value)
            except KeyError as e:
                value = "%s (%s)" % (value, "[UNKNOWN]")
        except AttributeError as e:
            # e.g. BAD_DATA
            pass
        except KeyError as e:
            pass

        f.write("    %s: %s\n" % (fieldname, value))


if __name__ == '__main__':
        serial_list = auto_detect_serial(preferred_list=['*FTDI*',"*Arduino_Mega_2560*", "*3D_Robotics*", "*USB_to_UART*", '*PX4*', '*FMU*'])
        for port in serial_list:
            print("%s" % port)