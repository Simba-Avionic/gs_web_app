import os
import time
import sys
import inspect
import threading

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports

# 1. Environment and Dialect Setup
os.environ["MAVLINK_DIALECT"] = "simba"

try:
    # Ensure the 'src' directory is in the path for imports
    sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))
    
    import src.simba as simba_module
    from src import mavutil
    
    # CRITICAL: Force mavutil to use your simba_module globally
    mavutil.mavlink = simba_module
    
except ImportError as e:
    print(f"Import Error: {e}. Ensure the 'src' folder contains simba.py and mavutil.py")

class MAVLinkSenderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("MAVLink Simba Tool - Send & Receive")
        self.root.geometry("750x900")

        # UI Styling
        style = ttk.Style()
        style.configure("Timestamp.TEntry", background='#e0e0e0', foreground='#707070')
   
        # App Variables
        self.mavlink_port = tk.StringVar()
        self.receiver_port = tk.StringVar()
        self.selected_message = tk.StringVar()
        self.field_vars = {}
        
        # Receiver State
        self.receiving = False
        self.receiver_thread = None

        # Notebook Setup
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill="both", expand=True, padx=10, pady=10)
        
        self.mavlink_tab = ttk.Frame(self.notebook)
        self.receiver_tab = ttk.Frame(self.notebook)
        
        self.notebook.add(self.mavlink_tab, text="Send Messages")
        self.notebook.add(self.receiver_tab, text="Received Messages")
        
        # Bottom Log Area
        self.setup_log_area()
        
        # Extract Data from Simba Dialect
        self.messages = self.get_mavlink_messages(simba_module)
        self.enums = self.get_mavlink_enums(simba_module)
        
        # Initialize UI Components
        self.setup_mavlink_tab()
        self.setup_receiver_tab()
        self.refresh_ports()
        
        # Cleanup on Close
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def get_mavlink_messages(self, module):
        messages = {}
        try:
            for name, obj in inspect.getmembers(module):
                if name.startswith('MAVLink_simba_') and name.endswith('_message') and inspect.isclass(obj):
                    msg_name = obj.msgname
                    fields = []
                    for field_name in obj.fieldnames:
                        field_idx = obj.fieldnames.index(field_name)
                        field_type = obj.fieldtypes[field_idx]
                        field_enum = obj.fieldenums_by_name.get(field_name)
                        fields.append({"name": field_name, "type": field_type, "enum": field_enum})
                    
                    messages[msg_name] = {"id": obj.id, "fields": fields, "class": obj}
            self.log(f"Dialect: Found {len(messages)} MAVLink messages.")
        except Exception as e:
            self.log(f"Error parsing messages: {e}")
        return messages
    
    def get_mavlink_enums(self, module):
        enums = {}
        try:
            module_enums = module.enums
            for enum_name, enum_dict in module_enums.items():
                entries = []
                for value, entry in enum_dict.items():
                    name = getattr(entry, 'name', str(entry))
                    entries.append((name, value))
                enums[enum_name] = entries
        except Exception as e:
            self.log(f"Error parsing enums: {e}")
        return enums

    def setup_mavlink_tab(self):
        # Port Selection
        port_frame = ttk.LabelFrame(self.mavlink_tab, text="Serial Port (Sender)")
        port_frame.pack(fill="x", padx=10, pady=5)
        ttk.Label(port_frame, text="Port:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.mavlink_port_combo = ttk.Combobox(port_frame, textvariable=self.mavlink_port)
        self.mavlink_port_combo.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        ttk.Button(port_frame, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=5, pady=5)
        
        # Message Selection
        message_frame = ttk.LabelFrame(self.mavlink_tab, text="Message Selection")
        message_frame.pack(fill="x", padx=10, pady=5)
        ttk.Label(message_frame, text="Message:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.message_combo = ttk.Combobox(message_frame, textvariable=self.selected_message, state="readonly")
        self.message_combo.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        
        message_names = sorted(self.messages.keys())
        self.message_combo['values'] = message_names
        if message_names: self.message_combo.current(0)
        self.message_combo.bind("<<ComboboxSelected>>", self.on_message_selected)
        
        # Dynamic Fields
        self.fields_frame = ttk.LabelFrame(self.mavlink_tab, text="Message Fields")
        self.fields_frame.pack(fill="both", expand=True, padx=10, pady=5)
        self.message_info = ttk.Label(self.fields_frame, text="Select a message to edit fields")
        self.message_info.pack(pady=10)
        
        ttk.Button(self.mavlink_tab, text="Send MAVLink Message", command=self.send_mavlink_message).pack(pady=10)
        self.root.after(100, self.on_message_selected)

    def setup_receiver_tab(self):
        # Configuration
        port_frame = ttk.LabelFrame(self.receiver_tab, text="Receiver Configuration")
        port_frame.pack(fill="x", padx=10, pady=5)
        ttk.Label(port_frame, text="Port:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.receiver_port_combo = ttk.Combobox(port_frame, textvariable=self.receiver_port)
        self.receiver_port_combo.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        
        self.listen_btn = ttk.Button(port_frame, text="Start Listening", command=self.toggle_receiver)
        self.listen_btn.grid(row=0, column=2, padx=5, pady=5)
        
        # Terminal View
        recv_frame = ttk.LabelFrame(self.receiver_tab, text="Live Telemetry")
        recv_frame.pack(fill="both", expand=True, padx=10, pady=5)
        self.recv_text = scrolledtext.ScrolledText(recv_frame, height=25, background="#000", foreground="#0f0", font=("Consolas", 10))
        self.recv_text.pack(fill="both", expand=True, padx=5, pady=5)
        ttk.Button(recv_frame, text="Clear Terminal", command=lambda: self.recv_text.delete(1.0, tk.END)).pack(pady=5)

    def setup_log_area(self):
        log_frame = ttk.LabelFrame(self.root, text="System Log")
        log_frame.pack(fill="x", side="bottom", padx=10, pady=5)
        self.log_text = scrolledtext.ScrolledText(log_frame, height=5)
        self.log_text.pack(fill="both", expand=True, padx=5, pady=5)

    def toggle_receiver(self):
        if not self.receiving:
            port = self.receiver_port.get()
            if not port: return
            self.receiving = True
            self.listen_btn.config(text="Stop Listening")
            self.receiver_thread = threading.Thread(target=self.run_receiver, args=(port,), daemon=True)
            self.receiver_thread.start()
        else:
            self.receiving = False
            self.listen_btn.config(text="Start Listening")

    def run_receiver(self, port):
        try:
            # Create connection and manually attach Simba MAVLink parser
            connection = mavutil.mavlink_connection(port, baud=57600)
            connection.mav = simba_module.MAVLink(connection)
            
            self.root.after(0, self.log, f"Receiver active on {port} (Simba Protocol)")

            while self.receiving:
                msg = connection.recv_match(blocking=False)
                if msg and msg.get_type() != 'BAD_DATA':
                    msg_type = msg.get_type().upper()
                    data = msg.to_dict()
                    data.pop('mavpackettype', None) # Remove redundant field
                    
                    output = f"[{time.strftime('%H:%M:%S')}] {msg_type}: {data}\n"
                    self.root.after(0, self.update_recv_ui, output)
                
                time.sleep(0.01) # Avoid high CPU usage
            connection.close()
        except Exception as e:
            self.root.after(0, self.log, f"Receiver Error: {e}")
            self.receiving = False
            self.root.after(0, self.listen_btn.config, {"text": "Start Listening"})

    def update_recv_ui(self, text):
        self.recv_text.insert(tk.END, text)
        self.recv_text.see(tk.END)

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports() if not p.device.startswith('/dev/ttyS')]
        self.mavlink_port_combo['values'] = ports
        self.receiver_port_combo['values'] = ports
        if ports:
            if not self.mavlink_port.get(): self.mavlink_port.set(ports[0])
            if not self.receiver_port.get(): self.receiver_port.set(ports[0])

    def on_message_selected(self, event=None):
        for widget in self.fields_frame.winfo_children(): widget.destroy()
        msg_name = self.selected_message.get()
        if msg_name not in self.messages: return
        
        message = self.messages[msg_name]
        fields_input_frame = ttk.Frame(self.fields_frame)
        fields_input_frame.pack(fill="both", expand=True, padx=5, pady=5)
        
        self.field_vars = {}
        for i, field in enumerate(message['fields']):
            fname, ftype, fenum = field['name'], field['type'], field['enum']
            is_ts = fname.lower() in ["timestamp", "time_usec", "time_boot_ms"]
            ttk.Label(fields_input_frame, text=f"{fname} ({ftype})").grid(row=i, column=0, padx=5, pady=5, sticky="w")
            
            var = tk.StringVar()
            if fenum:
                values = [e[0] for e in self.enums.get(fenum, [])]
                widget = ttk.Combobox(fields_input_frame, textvariable=var, values=values, state="readonly")
                if values: widget.current(0)
            else:
                var.set("[Auto]" if is_ts else ("0" if 'int' in ftype else "0.0"))
                widget = ttk.Entry(fields_input_frame, textvariable=var, state="disabled" if is_ts else "normal")
            
            widget.grid(row=i, column=1, padx=5, pady=5, sticky="w")
            self.field_vars[fname] = (var, ftype, fenum)

    def send_mavlink_message(self):
        port, msg_name = self.mavlink_port.get(), self.selected_message.get()
        if not port or not msg_name: return
        
        field_values = {}
        for fname, (var, ftype, fenum) in self.field_vars.items():
            val = var.get()
            if fname.lower() in ["timestamp", "time_usec", "time_boot_ms"]: 
                field_values[fname] = int(time.time() * 1000)
                continue
            try:
                if fenum: field_values[fname] = next(e[1] for e in self.enums.get(fenum, []) if e[0] == val)
                elif 'int' in ftype: field_values[fname] = int(val)
                else: field_values[fname] = float(val)
            except: 
                self.log(f"Invalid input for {fname}")
                return

        try:
            conn = mavutil.mavlink_connection(port, baud=57600)
            conn.mav = simba_module.MAVLink(conn) # Injection
            
            ordered_vals = [field_values.get(f['name'], 0) for f in self.messages[msg_name]['fields']]
            method_name = msg_name.lower() + "_encode"
            
            if hasattr(conn.mav, method_name):
                method = getattr(conn.mav, method_name)
                conn.mav.send(method(*ordered_vals))
                self.log(f"Sent {msg_name} to {port}")
            conn.close()
        except Exception as e: 
            self.log(f"Send Error: {e}")

    def log(self, message):
        if hasattr(self, 'log_text'):
            self.log_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {message}\n")
            self.log_text.see(tk.END)

    def on_close(self):
        self.receiving = False
        self.root.after(100, self.root.destroy)

if __name__ == "__main__":
    root = tk.Tk()
    app = MAVLinkSenderApp(root)
    root.mainloop()