import os
import time
import sys
import inspect
import threading

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports

# sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# import shared.utils as utils

os.environ["MAVLINK_DIALECT"] = "simba"
from mavlink.src.simba import *
from src import mavutil


class MAVLinkSenderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("MAVLink & Bit String Sender")
        self.root.geometry("600x800")

        style = ttk.Style()
        style.configure("Timestamp.TEntry", background='#e0e0e0', foreground='#707070')
   
        self.mavlink_port = tk.StringVar()
        self.bitstring_port = tk.StringVar()
        self.selected_message = tk.StringVar()
        self.bitstring = tk.StringVar(value="00000000")
        self.field_vars = {}

        self.command_port = tk.StringVar()
        self.response_port = tk.StringVar()
        self.command_state = tk.IntVar(value=0)
        self.response_state = tk.IntVar(value=0)
        self.auto_respond = tk.BooleanVar(value=False)
        self.responder_running = False
        self.responder_thread = None

        # Create notebook (tabs)
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill="both", expand=True, padx=10, pady=10)
        
        # Create tabs
        self.mavlink_tab = ttk.Frame(self.notebook)
        self.bitstring_tab = ttk.Frame(self.notebook)
        self.ack_tab = ttk.Frame(self.notebook) 
        
        self.notebook.add(self.mavlink_tab, text="MAVLink Messages")
        self.notebook.add(self.bitstring_tab, text="Bit String Sender")
        self.notebook.add(self.ack_tab, text="Test Ack") 
        
        # Add log area FIRST (before any methods that use logging)
        self.setup_log_area()
        
        # AFTER setting up logging, extract message info
        self.messages = self.get_mavlink_messages()
        self.enums = self.get_mavlink_enums()
        
        # Setup the tabs
        self.setup_mavlink_tab()
        self.setup_bitstring_tab()
        self.setup_ack_tab()  # Add setup for new tab
        
        # Refresh ports on startup
        self.refresh_ports()
    
    def get_mavlink_messages(self):
        """Extract message definitions from the simba module"""
        messages = {}
        
        # Look for all MAVLink_*_message classes in the module
        for name, obj in inspect.getmembers(sys.modules['simba']):
            if name.startswith('MAVLink_simba_') and name.endswith('_message') and inspect.isclass(obj):
                # Extract message name from class name (remove MAVLink_ prefix and _message suffix)
                msg_name = obj.msgname
                
                # Get message ID
                msg_id = obj.id
                
                # Get field information
                fields = []
                for field_name in obj.fieldnames:
                    # Find field type
                    field_idx = obj.fieldnames.index(field_name)
                    field_type = obj.fieldtypes[field_idx]
                    
                    # Check if field has an enum
                    field_enum = None
                    if field_name in obj.fieldenums_by_name:
                        field_enum = obj.fieldenums_by_name[field_name]
                    
                    fields.append({
                        "name": field_name,
                        "type": field_type,
                        "enum": field_enum
                    })
                
                # Add to messages dictionary
                messages[msg_name] = {
                    "id": msg_id,
                    "description": "", # Could extract from docstring but not critical
                    "fields": fields,
                    "class": obj  # Store the class for later use
                }
        
        self.log(f"Found {len(messages)} MAVLink messages")
        return messages
    
    def get_mavlink_enums(self):
        """Extract enum definitions from the simba module"""
        enums = {}
        
        # Access the enums dictionary directly from the module
        module_enums = sys.modules['simba'].enums
        
        for enum_name, enum_dict in module_enums.items():
            entries = []
            for value, entry in enum_dict.items():
                if isinstance(entry, EnumEntry):
                    entries.append((entry.name, value))
            enums[enum_name] = entries
        
        self.log(f"Found {len(enums)} MAVLink enums")
        return enums
    
    def setup_ack_tab(self):
        """Setup the ACK testing tab"""
        # Command sender frame
        command_frame = ttk.LabelFrame(self.ack_tab, text="Command Sender")
        command_frame.pack(fill="x", padx=10, pady=5)
        
        # Port selection for command
        ttk.Label(command_frame, text="Command Port:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.command_port_combo = ttk.Combobox(command_frame, textvariable=self.command_port)
        self.command_port_combo.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        
        # State selection for command
        ttk.Label(command_frame, text="State Value:").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        
        # Create a dropdown for state selection if SIMBA_ROCKET_STATE enum exists
        state_enum = self.enums.get('SIMBA_ROCKET_STATE', [])
        if state_enum:
            state_values = [(entry[0], entry[1]) for entry in state_enum]
            state_frame = ttk.Frame(command_frame)
            state_frame.grid(row=1, column=1, padx=5, pady=5, sticky="w")
            
            state_var = tk.StringVar()
            state_combo = ttk.Combobox(state_frame, textvariable=state_var, 
                                    values=[s[0] for s in state_values], state="readonly", width=20)
            if state_values:
                state_combo.current(0)
                self.command_state.set(state_values[0][1])
            state_combo.pack(side=tk.LEFT, padx=5)
            
            # Update integer value when dropdown changes
            def on_state_selected(event):
                selected = state_var.get()
                for name, value in state_values:
                    if name == selected:
                        self.command_state.set(value)
                        break
            state_combo.bind("<<ComboboxSelected>>", on_state_selected)
            
            # Also show numeric value
            ttk.Label(state_frame, textvariable=tk.StringVar(value="Value:")).pack(side=tk.LEFT, padx=5)
            ttk.Entry(state_frame, textvariable=self.command_state, width=5).pack(side=tk.LEFT)
        else:
            # Fallback to simple entry if enum not available
            ttk.Entry(command_frame, textvariable=self.command_state, width=10).grid(
                row=1, column=1, padx=5, pady=5, sticky="w")
        
        # Send command button
        ttk.Button(command_frame, text="Send Command", 
                command=self.send_command).grid(row=2, column=0, columnspan=2, pady=10)
        
        # Response frame
        response_frame = ttk.LabelFrame(self.ack_tab, text="Response Handler")
        response_frame.pack(fill="x", padx=10, pady=5)
        
        # Port selection for response
        ttk.Label(response_frame, text="Response Port:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.response_port_combo = ttk.Combobox(response_frame, textvariable=self.response_port)
        self.response_port_combo.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        
        # Auto-respond checkbox
        auto_resp_frame = ttk.Frame(response_frame)
        auto_resp_frame.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="w")
        
        ttk.Checkbutton(auto_resp_frame, text="Auto-respond to commands", 
                        variable=self.auto_respond).pack(side=tk.LEFT, padx=5)
        
        # State for response (same as command by default)
        # In setup_ack_tab(), modify the response frame:

        # State for response (same as command by default)
        ttk.Label(response_frame, text="Response State:").grid(row=2, column=0, padx=5, pady=5, sticky="w")
        ttk.Entry(response_frame, textvariable=self.response_state, width=10).grid(
            row=2, column=1, padx=5, pady=5, sticky="w")

        # Add status selection for response
        ttk.Label(response_frame, text="Response Status:").grid(row=3, column=0, padx=5, pady=5, sticky="w")

        # Create status variable and dropdown
        self.response_status = tk.StringVar(value="SIMBA_OK")
        status_enum = self.enums.get('SIMBA_STATUS', [])
        if status_enum:
            status_combo = ttk.Combobox(response_frame, textvariable=self.response_status, 
                                values=[s[0] for s in status_enum], state="readonly", width=20)
            status_combo.current(0)  # Default to SIMBA_OK
            status_combo.grid(row=3, column=1, padx=5, pady=5, sticky="w")
        else:
            ttk.Entry(response_frame, textvariable=self.response_status, width=10).grid(
                row=3, column=1, padx=5, pady=5, sticky="w")

        # Copy button to copy command state to response state
        ttk.Button(response_frame, text="Use Command State", 
                command=lambda: self.response_state.set(self.command_state.get())).grid(
            row=4, column=0, columnspan=2, pady=5)

        # Send response button
        ttk.Button(response_frame, text="Send Response", 
                command=self.send_response).grid(row=5, column=0, columnspan=2, pady=10)
        
        # Instruction text
        info_frame = ttk.LabelFrame(self.ack_tab, text="Instructions")
        info_frame.pack(fill="x", padx=10, pady=5)
        
        info_text = """This tab simulates a command-acknowledgment interaction:
        1. Select different ports for command and response
        2. Configure the state value for the command
        3. Send the command using 'Send Command' button
        4. Either manually send a response, or enable 'Auto-respond'
        5. The response will use the state value specified in 'Response State'

        With auto-respond enabled, the system will automatically send an acknowledgment
        whenever a command is received on the response port."""
        
        info_label = ttk.Label(info_frame, text=info_text, justify="left", wraplength=580)
        info_label.pack(padx=10, pady=5, anchor="w")
        
        # Refresh button
        ttk.Button(self.ack_tab, text="Refresh Ports", command=self.refresh_ports).pack(pady=10)

        # After setting up port dropdowns in setup_ack_tab
        if len(self.command_port_combo['values']) > 1:
            # Set command port to first value and response port to second value
            self.command_port.set(self.command_port_combo['values'][0])
            self.response_port.set(self.command_port_combo['values'][1])
        
        def on_auto_respond_changed(*args):
            if self.auto_respond.get():
                self.toggle_auto_responder()
            else:
                self.responder_running = False

        self.auto_respond.trace_add("write", on_auto_respond_changed)
    

    def send_command(self):
        """Send a SIMBA_CMD_CHANGE_STATE message"""
        port = self.command_port.get()
        state = self.command_state.get()
        
        if not port:
            self.log("Error: No command port selected")
            return
        
        try:
            connection = mavutil.mavlink_connection(port, baud=57600)
            self.log(f"Connected to command port {port}")
            
            # Encode and send the message
            msg = connection.mav.simba_cmd_change_state_encode(state)
            connection.mav.send(msg)
            
            self.log(f"Sent SIMBA_CMD_CHANGE_STATE with state={state}")
            connection.close()
            
        except Exception as e:
            self.log(f"Error sending command: {e}")

    def send_response(self):
        """Send a SIMBA_ACK message"""
        port = self.response_port.get()
        state = self.response_state.get()
        status_name = self.response_status.get()
        
        if not port:
            self.log("Error: No response port selected")
            return
        
        # Get numeric status value from enum name
        status = 0  # Default to SIMBA_OK
        for name, value in self.enums.get('SIMBA_STATUS', []):
            if name == status_name:
                status = value
                break
        
        try:
            connection = mavutil.mavlink_connection(port, baud=57600)
            self.log(f"Connected to response port {port}")
            
            # Get SIMBA_OK value (default 0)
            status = 0  # Default to SIMBA_OK
            for name, value in self.enums.get('SIMBA_STATUS', []):
                if name == 'SIMBA_OK':
                    status = value
                    break
            
            # Encode and send the ack message with both state and status parameters
            msg = connection.mav.simba_ack_encode(state, status)
            connection.mav.send(msg)
            
            self.log(f"Sent SIMBA_ACK with state={state}, status=SIMBA_OK({status})")
            connection.close()
            
        except Exception as e:
            self.log(f"Error sending response: {e}")

    def toggle_auto_responder(self):
        """Toggle the automatic responder"""
        if self.responder_running:
            self.responder_running = False
            self.log("Auto-responder stopped")
            if self.responder_thread:
                self.responder_thread.join(timeout=1.0)
                self.responder_thread = None
        else:
            if not self.response_port.get():
                self.log("Error: No response port selected")
                self.auto_respond.set(False)
                return
                
            self.responder_running = True
            self.responder_thread = threading.Thread(target=self.run_auto_responder, daemon=True)
            self.responder_thread.start()
            self.log("Auto-responder started")

    def run_auto_responder(self):
        """Thread function to automatically respond to commands"""
        port = self.response_port.get()
        
        try:
            connection = mavutil.mavlink_connection(port, baud=57600)
            self.log(f"Auto-responder listening on {port}")
            
            # Get SIMBA_OK value (default 0)
            status = 0  # Default to SIMBA_OK
            for name, value in self.enums.get('SIMBA_STATUS', []):
                if name == 'SIMBA_OK':
                    status = value
                    break
            
            while self.responder_running:
                # Wait for a command message with a short timeout
                msg = connection.recv_match(type='simba_cmd_change_state', blocking=True, timeout=0.5)
                
                if msg is not None:
                    self.log(f"Auto-responder received command with state={msg.new_state}")
                    
                    # Use the received state for the response
                    self.response_state.set(msg.new_state)
                    
                    # Send the ack with proper status
                    ack_msg = connection.mav.simba_ack_encode(msg.new_state, status)
                    connection.mav.send(ack_msg)
                    self.log(f"Auto-responder sent ACK with state={msg.new_state}, status=SIMBA_OK({status})")
                
                # Small sleep to prevent high CPU usage
                time.sleep(0.01)
                
            connection.close()
            
        except Exception as e:
            self.log(f"Error in auto-responder: {e}")
            self.responder_running = False
            self.auto_respond.set(False)

    def setup_mavlink_tab(self):
        """Setup the MAVLink message sender tab"""
        # Port selection frame
        port_frame = ttk.LabelFrame(self.mavlink_tab, text="Serial Port")
        port_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(port_frame, text="Port:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.mavlink_port_combo = ttk.Combobox(port_frame, textvariable=self.mavlink_port)
        self.mavlink_port_combo.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        
        ttk.Button(port_frame, text="Refresh", command=self.refresh_ports).grid(
            row=0, column=2, padx=5, pady=5)
        
        # Message selection frame
        message_frame = ttk.LabelFrame(self.mavlink_tab, text="Message Selection")
        message_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(message_frame, text="Message:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.message_combo = ttk.Combobox(message_frame, textvariable=self.selected_message, state="readonly")
        self.message_combo.grid(row=0, column=1, padx=5, pady=5, sticky="w")
               
        message_names = [name for name in sorted(self.messages.keys())]
        self.message_combo['values'] = message_names
        if message_names:
            self.message_combo.current(0)
        
        self.message_combo.bind("<<ComboboxSelected>>", self.on_message_selected)
        
        # Create fields frame (will be populated when message is selected)
        self.fields_frame = ttk.LabelFrame(self.mavlink_tab, text="Message Fields")
        self.fields_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        # Message info label
        self.message_info = ttk.Label(self.fields_frame, text="Select a message to view fields")
        self.message_info.pack(pady=10)
        
        # Send button
        ttk.Button(self.mavlink_tab, text="Send MAVLink Message", command=self.send_mavlink_message).pack(
            pady=10)
        
        # Trigger initial message selection
        self.root.after(100, self.on_message_selected)
    
    def setup_bitstring_tab(self):
        """Setup the bit string sender tab"""
        # Port selection frame
        port_frame = ttk.LabelFrame(self.bitstring_tab, text="Serial Port")
        port_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(port_frame, text="Port:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.bitstring_port_combo = ttk.Combobox(port_frame, textvariable=self.bitstring_port)
        self.bitstring_port_combo.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        
        ttk.Button(port_frame, text="Refresh", command=self.refresh_ports).grid(
            row=0, column=2, padx=5, pady=5)
        
        # Bit mapping information
        info_frame = ttk.LabelFrame(self.bitstring_tab, text="Bit Mapping Information")
        info_frame.pack(fill="x", padx=10, pady=5)
        
        info_text = """Bit positions correspond to the following actions:
        Bit 0: Arm/Disarm - Arm/disarm the rocket systems
        Bit 1: Ignition - Trigger ignition sequence
        Bit 2: Tank_vent - Control rocket tank vent valve
        Bit 3: Main_valve - Control ground segment main valve
        Bit 4: Hose_vent- Control ground segment hose vent
        Bit 5: Decoupler - Activate decoupler
        Bit 6: N2O_He_switch - Switch between N2O and Helium
        Bit 7: Abort - Emergency abort sequence
        
        Set bit to 1 to activate the corresponding function."""
        
        info_label = ttk.Label(info_frame, text=info_text, justify="left", wraplength=600)
        info_label.pack(padx=10, pady=5, anchor="w")
        
        # Bit string input frame
        bitstring_frame = ttk.LabelFrame(self.bitstring_tab, text="Bit String (8 bits)")
        bitstring_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(bitstring_frame, text="Binary:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        ttk.Entry(bitstring_frame, textvariable=self.bitstring, width=10).grid(
            row=0, column=1, padx=5, pady=5, sticky="w")
        
        # Bit toggle buttons
        toggle_frame = ttk.Frame(bitstring_frame)
        toggle_frame.grid(row=1, column=0, columnspan=2, padx=5, pady=10)
        
        # Define bit descriptions
        bit_descriptions = [
            "abort",
            "N2O_He_switch",
            "decoupler",
            "hose_vent",
            "main_valve",
            "tank_vent",
            "ignition",
            "arm/disarm"
        ]
        
        # Labels for bit positions
        for i in range(8):
            ttk.Label(toggle_frame, text=f"Bit {7-i}").grid(row=0, column=i, padx=5, pady=2)
            ttk.Label(toggle_frame, text=bit_descriptions[i], font=("", 8)).grid(row=1, column=i, padx=5, pady=0)
        
        # Toggle buttons
        self.bit_buttons = []
        for i in range(8):
            btn = ttk.Button(toggle_frame, text="0", width=3, 
                            command=lambda idx=7-i: self.toggle_bit(idx))
            btn.grid(row=2, column=i, padx=5, pady=2)
            self.bit_buttons.append(btn)
        
        # Category labels (color-coded)
        for i in range(8):
            category = "rocket" if i >= 5 else "gs" if i >= 1 else "abort"
            color = "#FFD700" if category == "rocket" else "#90EE90" if category == "gs" else "#FF6347"  # Gold, LightGreen, Tomato
            
            category_label = tk.Label(toggle_frame, text=category, bg=color, relief="ridge", font=("", 7))
            category_label.grid(row=3, column=i, padx=5, pady=2, sticky="ew")
        
        # Update bit buttons from initial value
        self.update_bit_buttons()
        
        # Send button
        ttk.Button(self.bitstring_tab, text="Send Bit String", command=self.send_bitstring).pack(
            pady=10)
    
    def setup_log_area(self):
        """Setup the logging area at the bottom of the window"""
        log_frame = ttk.LabelFrame(self.root, text="Log")
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        # Create scrolled text widget for logs
        self.log_text = scrolledtext.ScrolledText(log_frame, height=10)
        self.log_text.pack(fill="both", expand=True, padx=5, pady=5)
        
        # Clear log button
        ttk.Button(log_frame, text="Clear Log", command=lambda: self.log_text.delete(1.0, tk.END)).pack(
            pady=5)
    
    def refresh_ports(self):
        """Refresh the list of available serial ports"""
        ports = [port.device for port in serial.tools.list_ports.comports() 
                if not port.device.startswith('/dev/ttyS')]
        
        # Update MAVLink tab port dropdown
        self.mavlink_port_combo['values'] = ports
        
        # Update Bitstring tab port dropdown
        self.bitstring_port_combo['values'] = ports
        
        # Update Test Ack tab port dropdowns
        self.command_port_combo['values'] = ports
        self.response_port_combo['values'] = ports
        
        # Set default values if none selected
        if ports:
            if not self.mavlink_port.get() or self.mavlink_port.get() not in ports:
                self.mavlink_port.set(ports[0])
            if not self.bitstring_port.get() or self.bitstring_port.get() not in ports:
                self.bitstring_port.set(ports[0])
            if not self.command_port.get() or self.command_port.get() not in ports:
                self.command_port.set(ports[0])
            if not self.response_port.get() or self.response_port.get() not in ports:
                self.response_port.set(ports[0])
        
        self.log(f"Found {len(ports)} serial ports (excluding ttyS* ports)")
    
    def on_message_selected(self, event=None):
        """Handle message selection event"""
        # Clear previous fields
        for widget in self.fields_frame.winfo_children():
            widget.destroy()
        
        message_name = self.selected_message.get()
        if not message_name or message_name not in self.messages:
            return
        
        message = self.messages[message_name]
        
        # Show message info
        info_text = f"Message ID: {message['id']}"
        ttk.Label(self.fields_frame, text=info_text, wraplength=600).pack(pady=5)
        
        # Create a frame for fields
        fields_input_frame = ttk.Frame(self.fields_frame)
        fields_input_frame.pack(fill="both", expand=True, padx=5, pady=5)
        
        # Add field inputs
        self.field_vars = {}  # Reset field variables
        
        for i, field in enumerate(message['fields']):
            field_name = field['name']
            field_type = field['type']
            field_enum = field['enum']
            
            # Check if this is a timestamp field
            is_timestamp = field_name.lower() in ["timestamp", "time_usec", "time_boot_ms", "time_unix_usec"]
            
            # Add a label for this field
            label_text = f"{field_name} ({field_type})"
            if is_timestamp:
                label_text += " [AUTO]"  # Indicate auto-filled
                
            ttk.Label(fields_input_frame, text=label_text).grid(
                row=i, column=0, padx=5, pady=5, sticky="w")
            
            # Create variable and widget based on field type
            if field_enum:
                # For enum fields, create a dropdown
                var = tk.StringVar()
                values = [entry[0] for entry in self.enums.get(field_enum, [])]
                combo = ttk.Combobox(fields_input_frame, textvariable=var, values=values, 
                                    state="readonly" if not is_timestamp else "disabled")
                if values:
                    combo.current(0)
                combo.grid(row=i, column=1, padx=5, pady=5, sticky="w")
                self.field_vars[field_name] = (var, field_type, field_enum)
                
                # Gray out timestamp fields
                if is_timestamp:
                    combo.configure(state="disabled")
            else:
                # For non-enum fields, create a text entry
                var = tk.StringVar()
                
                # Set default values based on type
                if is_timestamp:
                    # For timestamp fields, show placeholder
                    if 'uint64' in field_type or field_name.lower() == "time_unix_usec":
                        var.set("[Auto: μs]")
                    elif field_name.lower() == "time_boot_ms":
                        var.set("[Auto: ms]")
                    else:
                        var.set("[Auto: s]")
                elif 'int' in field_type:
                    var.set("0")
                elif 'float' in field_type:
                    var.set("0.0")
                elif 'char' in field_type:
                    var.set("")
                
                # Create the entry widget
                entry = ttk.Entry(fields_input_frame, textvariable=var, 
                                state="normal" if not is_timestamp else "disabled")
                entry.grid(row=i, column=1, padx=5, pady=5, sticky="w")
                
                # Set a gray background for timestamp fields
                if is_timestamp:
                    entry.configure(style="Timestamp.TEntry")
                
                self.field_vars[field_name] = (var, field_type, None)
    
    def toggle_bit(self, index):
        """Toggle a specific bit in the bit string"""
        current = self.bitstring.get()
        if len(current) != 8 or not all(c in '01' for c in current):
            # Reset to valid bit string if invalid
            current = "00000000"
        
        # Toggle the bit at the specified index
        new_bit = '1' if current[index] == '0' else '0'
        new_string = current[:index] + new_bit + current[index+1:]
        self.bitstring.set(new_string)
        
        # Update button text
        self.update_bit_buttons()
    
    def update_bit_buttons(self):
        """Update the bit toggle buttons to reflect current bit string"""
        current = self.bitstring.get()
        if len(current) != 8:
            current = "00000000"
            self.bitstring.set(current)
        
        for i, btn in enumerate(self.bit_buttons):
            btn.config(text=current[7-i])  # Reversed to show MSB on left
    
    def send_mavlink_message(self):
        """Send a MAVLink message"""
        port = self.mavlink_port.get()
        message_name = self.selected_message.get()
        
        if not port:
            self.log("Error: No port selected")
            return
        
        if not message_name:
            self.log("Error: No message selected")
            return
        
        field_values = {}
        for field_name, (var, field_type, field_enum) in self.field_vars.items():
            value = var.get()

            if field_name.lower() == "timestamp":
                utc_time = time.time()  # This is already UTC epoch time
                field_values[field_name] = int(utc_time)
                continue
                            
            try:
                # Convert string to appropriate type
                if field_enum:
                    # Find the enum value from the name
                    enum_entries = self.enums.get(field_enum, [])
                    for entry_name, entry_value in enum_entries:
                        if entry_name == value:
                            field_values[field_name] = entry_value
                            break
                elif 'uint' in field_type or 'int' in field_type:
                    field_values[field_name] = int(value)
                elif 'float' in field_type:
                    field_values[field_name] = float(value)
                else:
                    field_values[field_name] = value
            except ValueError:
                self.log(f"Error: Invalid value for field {field_name}")
                return
        
        try:
            connection = mavutil.mavlink_connection(port, baud=57600)
            self.log(f"Connected to {port}")
            
            # Get message definition to determine field order
            message_info = self.messages.get(message_name)
            if not message_info:
                self.log(f"Error: Message definition not found for {message_name}")
                return
            
            # Build ordered list of field values based on field definitions
            ordered_values = []
            for field in message_info['fields']:
                field_name = field['name']
                ordered_values.append(field_values.get(field_name, 0))  # Default to 0 if missing
            
            # Log what we're about to send
            field_summary = ', '.join([f"{field['name']}={field_values.get(field['name'], 0)}" 
                                    for field in message_info['fields']])
            self.log(f"Sending {message_name} with: {field_summary}")
            
            # Dynamically get the encode method for this message type
            # Convert message name to lowercase with underscores
            method_name = message_name.lower() + "_encode"
            
            # Check if the method exists
            if hasattr(connection.mav, method_name):
                # Get the method and call it with our field values
                encode_method = getattr(connection.mav, method_name)
                msg = encode_method(*ordered_values)
                
                # Send the message
                connection.mav.send(msg)
                self.log(f"Successfully sent {message_name} message")
            else:
                self.log(f"Error: Method {method_name} not found in MAVLink dialect")
                
            connection.close()
            
        except Exception as e:
            self.log(f"Error sending MAVLink message: {e}")
    
    def send_bitstring(self):
        """Send a bit string via serial port"""
        port = self.bitstring_port.get()
        bit_string = self.bitstring.get()
        
        if not port:
            self.log("Error: No port selected")
            return
        
        if len(bit_string) != 8 or not all(c in '01' for c in bit_string):
            self.log("Error: Invalid bit string, must be 8 bits (0s and 1s)")
            return
        
        try:
            with serial.Serial(port, baudrate=57600, timeout=0) as ser:
                ser.write(bit_string.encode('utf-8') + b'\n')
                self.log(f"Sent bit string {bit_string} to {port}")
        
        except Exception as e:
            self.log(f"Error sending bit string: {e}")
    
    def log(self, message):
        """Add a message to the log area with timestamp"""
        timestamp = time.strftime("%H:%M:%S")
        
        # Only use log_text if it exists
        if hasattr(self, 'log_text'):
            self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
            self.log_text.see(tk.END)  # Scroll to the end
        else:
            # Print to console if log_text is not available yet
            print(f"[{timestamp}] {message}")


if __name__ == "__main__":
    root = tk.Tk()
    app = MAVLinkSenderApp(root)
    root.mainloop()