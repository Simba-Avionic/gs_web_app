import os
import time
import sys
import inspect

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import shared.utils as utils

os.environ["MAVLINK_DIALECT"] = "simba"
# simba = utils.patch_mavlink_dialect()
from simba import *
from src import mavutil


class MAVLinkSenderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("MAVLink & Bit String Sender")
        self.root.geometry("600x800")       
   
        self.mavlink_port = tk.StringVar()
        self.bitstring_port = tk.StringVar()
        self.selected_message = tk.StringVar()
        self.bitstring = tk.StringVar(value="00000000")
        self.field_vars = {}

        # Create notebook (tabs)
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill="both", expand=True, padx=10, pady=10)
        
        # Create tabs
        self.mavlink_tab = ttk.Frame(self.notebook)
        self.bitstring_tab = ttk.Frame(self.notebook)
        
        self.notebook.add(self.mavlink_tab, text="MAVLink Messages")
        self.notebook.add(self.bitstring_tab, text="Bit String Sender")
        
        # Add log area FIRST (before any methods that use logging)
        self.setup_log_area()
        
        # AFTER setting up logging, extract message info
        self.messages = self.get_mavlink_messages()
        self.enums = self.get_mavlink_enums()
        
        # Setup the tabs
        self.setup_mavlink_tab()
        self.setup_bitstring_tab()
        
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
        
        # Populate message dropdown - filter out command messages
        message_names = [name for name in sorted(self.messages.keys()) if name in 
                        ["SIMBA_CMD_CHANGE_STATE", "SIMBA_ACTUATOR_CMD", "SIMBA_ACK"]]
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
        ports = [port.device for port in serial.tools.list_ports.comports()]
        
        self.mavlink_port_combo['values'] = ports
        self.bitstring_port_combo['values'] = ports
        
        if ports:
            if not self.mavlink_port.get() or self.mavlink_port.get() not in ports:
                self.mavlink_port.set(ports[0])
            if not self.bitstring_port.get() or self.bitstring_port.get() not in ports:
                self.bitstring_port.set(ports[0])
        
        self.log(f"Found {len(ports)} serial ports")
    
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
            
            ttk.Label(fields_input_frame, text=f"{field_name} ({field_type}):").grid(
                row=i, column=0, padx=5, pady=5, sticky="w")
            
            # Create variable and widget based on field type
            if field_enum:
                # For enum fields, create a dropdown
                var = tk.StringVar()
                values = [entry[0] for entry in self.enums.get(field_enum, [])]
                combo = ttk.Combobox(fields_input_frame, textvariable=var, values=values, state="readonly")
                if values:
                    combo.current(0)
                combo.grid(row=i, column=1, padx=5, pady=5, sticky="w")
                self.field_vars[field_name] = (var, field_type, field_enum)
            else:
                # For non-enum fields, create a text entry
                var = tk.StringVar()
                # Set default values based on type
                if 'int' in field_type:
                    var.set("0")
                elif 'float' in field_type:
                    var.set("0.0")
                elif 'char' in field_type:
                    var.set("")
                
                entry = ttk.Entry(fields_input_frame, textvariable=var)
                entry.grid(row=i, column=1, padx=5, pady=5, sticky="w")
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
        
        # Prepare field values
        field_values = {}
        for field_name, (var, field_type, field_enum) in self.field_vars.items():
            value = var.get()
            
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
            # Create and send MAVLink message
            connection = mavutil.mavlink_connection(port, baud=57600)
            self.log(f"Connected to {port}")
            
            # Wait for heartbeat
            self.log("Waiting for heartbeat...")
            # connection.wait_heartbeat(timeout=5)
            # self.log("Heartbeat received")
            
            # Use the encode function directly from simba.py
            if message_name == "SIMBA_CMD_CHANGE_STATE":
                msg = connection.mav.simba_cmd_change_state_encode(
                    field_values.get("new_state", 0)
                )
                self.log(f"Sending SIMBA_CMD_CHANGE_STATE with state={field_values.get('new_state', 0)}")
                
            elif message_name == "SIMBA_ACTUATOR_CMD":
                msg = connection.mav.simba_actuator_cmd_encode(
                    field_values.get("actuator_id", 0),
                    field_values.get("value", 0)
                )
                self.log(f"Sending SIMBA_ACTUATOR_CMD with id={field_values.get('actuator_id', 0)}, value={field_values.get('value', 0)}")
                
            elif message_name == "SIMBA_ACK":
                msg = connection.mav.simba_ack_encode(
                    field_values.get("state", 0),
                    field_values.get("status", 0)
                )
                self.log(f"Sending SIMBA_ACK with state={field_values.get('state', 0)}, status={field_values.get('status', 0)}")
                
            else:
                self.log(f"Unsupported message type: {message_name}")
                return
            
            # Send the message
            connection.mav.send(msg)
            self.log(f"Sent {message_name} message with fields: {field_values}")
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
            # Convert bit string to byte
            byte_value = int(bit_string, 2).to_bytes(1, byteorder='big')
            
            # Send via serial port
            with serial.Serial(port, 9600, timeout=1) as ser:
                ser.write(byte_value)
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