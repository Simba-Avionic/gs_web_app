#!/usr/bin/env python3
"""
Config Tinker - Simple Tkinter editor for config.json
Edits workspace root file: config.json (creates timestamped backup on save).
"""

import json
import os
import shutil
import time
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog, filedialog

ROOT_CONFIG = "config.json"


def load_config(path=ROOT_CONFIG):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def backup_config(path=ROOT_CONFIG):
    ts = time.strftime("%Y%m%d-%H%M%S")
    dst = f"{path}.bak-{ts}"
    shutil.copy2(path, dst)
    return dst


def save_config(data, path=ROOT_CONFIG):
    backup = backup_config(path)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4, ensure_ascii=False)
    return backup


def ensure_list_of_length(val, length):
    if not isinstance(val, list):
        return [val] + [None] * (length - 1)
    return val + [None] * max(0, length - len(val))


class ConfigTinker(tk.Tk):
    def __init__(self, config_path=ROOT_CONFIG):
        super().__init__()
        self.title("Config Tinker")
        self.geometry("1000x600")
        self.config_path = config_path
        self.data = {}
        self.selected_side = None  # "topic" or "msgdef"
        self.selected_index = None

        self._build_ui()
        self._load()

    def _build_ui(self):
        # Paned
        paned = ttk.Panedwindow(self, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True)

        left_frame = ttk.Frame(paned, width=300)
        right_frame = ttk.Frame(paned)
        paned.add(left_frame, weight=1)
        paned.add(right_frame, weight=3)

        # Tabs for Topics / MsgDefs
        tab = ttk.Notebook(left_frame)
        tab.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        self.topics_frame = ttk.Frame(tab)
        self.msgdefs_frame = ttk.Frame(tab)
        tab.add(self.topics_frame, text="Topics")
        tab.add(self.msgdefs_frame, text="MsgDefs")

        # Topics list
        self.topics_list = tk.Listbox(self.topics_frame)
        self.topics_list.pack(fill=tk.BOTH, expand=True, padx=6, pady=(6, 0))
        self.topics_list.bind("<<ListboxSelect>>", self.on_topic_select)

        tbtn = ttk.Frame(self.topics_frame)
        tbtn.pack(fill=tk.X, padx=6, pady=6)
        ttk.Button(tbtn, text="Add Topic", command=self.add_topic).pack(side=tk.LEFT)
        ttk.Button(tbtn, text="Delete Topic", command=self.delete_topic).pack(side=tk.LEFT, padx=6)
        ttk.Button(tbtn, text="New From File...", command=self.load_topic_from_file).pack(side=tk.LEFT, padx=6)

        # MsgDefs list
        self.msgdefs_list = tk.Listbox(self.msgdefs_frame)
        self.msgdefs_list.pack(fill=tk.BOTH, expand=True, padx=6, pady=(6, 0))
        self.msgdefs_list.bind("<<ListboxSelect>>", self.on_msgdef_select)

        mbtn = ttk.Frame(self.msgdefs_frame)
        mbtn.pack(fill=tk.X, padx=6, pady=6)
        ttk.Button(mbtn, text="Add MsgDef", command=self.add_msgdef).pack(side=tk.LEFT)
        ttk.Button(mbtn, text="Delete MsgDef", command=self.delete_msgdef).pack(side=tk.LEFT, padx=6)

        # Right: editor for msg_fields
        hdr = ttk.Frame(right_frame)
        hdr.pack(fill=tk.X, padx=6, pady=6)
        self.header_label = ttk.Label(hdr, text="Select a Topic or MsgDef to edit")
        self.header_label.pack(side=tk.LEFT)

        # Fields list treeview
        cols = ("index", "kind", "val_name", "type_or_def")
        self.fields_tree = ttk.Treeview(right_frame, columns=cols, show="headings", selectmode="browse")
        self.fields_tree.heading("index", text="#")
        self.fields_tree.heading("kind", text="Kind")
        self.fields_tree.heading("val_name", text="Val Name")
        self.fields_tree.heading("type_or_def", text="Type / MsgDef")
        self.fields_tree.column("index", width=40, anchor="center")
        self.fields_tree.column("kind", width=80)
        self.fields_tree.column("val_name", width=200)
        self.fields_tree.column("type_or_def", width=200)
        self.fields_tree.pack(fill=tk.BOTH, expand=True, padx=6, pady=(0, 6))
        self.fields_tree.bind("<<TreeviewSelect>>", self.on_field_select)

        # Field editor form
        form = ttk.Frame(right_frame)
        form.pack(fill=tk.X, padx=6, pady=(0, 6))

        # Kind: type or msg_def (radiobutton)
        self.field_kind = tk.StringVar(value="type")
        kf = ttk.Frame(form)
        kf.pack(fill=tk.X, pady=2)
        ttk.Label(kf, text="Field kind:").pack(side=tk.LEFT)
        ttk.Radiobutton(kf, text="type", variable=self.field_kind, value="type").pack(side=tk.LEFT, padx=4)
        ttk.Radiobutton(kf, text="msg_def", variable=self.field_kind, value="msg_def").pack(side=tk.LEFT, padx=4)

        # val_name
        row = ttk.Frame(form); row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text="val_name:", width=12).pack(side=tk.LEFT)
        self.val_name_var = tk.StringVar()
        ttk.Entry(row, textvariable=self.val_name_var).pack(side=tk.LEFT, fill=tk.X, expand=True)

        # type / msg_def
        row = ttk.Frame(form); row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text="type/msg_def:", width=12).pack(side=tk.LEFT)
        self.type_var = tk.StringVar()
        ttk.Entry(row, textvariable=self.type_var).pack(side=tk.LEFT, fill=tk.X, expand=True)

        # alt_name
        row = ttk.Frame(form); row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text="alt_name:", width=12).pack(side=tk.LEFT)
        self.alt_var = tk.StringVar()
        ttk.Entry(row, textvariable=self.alt_var).pack(side=tk.LEFT, fill=tk.X, expand=True)

        # unit
        row = ttk.Frame(form); row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text="unit:", width=12).pack(side=tk.LEFT)
        self.unit_var = tk.StringVar()
        ttk.Entry(row, textvariable=self.unit_var).pack(side=tk.LEFT, fill=tk.X, expand=True)

        # display
        row = ttk.Frame(form); row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text="display:", width=12).pack(side=tk.LEFT)
        self.display_var = tk.StringVar()
        ttk.Entry(row, textvariable=self.display_var).pack(side=tk.LEFT, fill=tk.X, expand=True)

        # range (two fields)
        row = ttk.Frame(form); row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text="range (min,max):", width=12).pack(side=tk.LEFT)
        self.range_min_var = tk.StringVar()
        self.range_max_var = tk.StringVar()
        ttk.Entry(row, width=10, textvariable=self.range_min_var).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Entry(row, width=10, textvariable=self.range_max_var).pack(side=tk.LEFT)

        # mapping (comma separated)
        row = ttk.Frame(form); row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text="mapping:", width=12).pack(side=tk.LEFT)
        self.mapping_var = tk.StringVar()
        ttk.Entry(row, textvariable=self.mapping_var).pack(side=tk.LEFT, fill=tk.X, expand=True)

        # controls / offset (free text)
        row = ttk.Frame(form); row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text="controls:", width=12).pack(side=tk.LEFT)
        self.controls_var = tk.StringVar()
        ttk.Entry(row, textvariable=self.controls_var).pack(side=tk.LEFT, fill=tk.X, expand=True)

        row = ttk.Frame(form); row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text="offset (comma):", width=12).pack(side=tk.LEFT)
        self.offset_var = tk.StringVar()
        ttk.Entry(row, textvariable=self.offset_var).pack(side=tk.LEFT, fill=tk.X, expand=True)

        # Buttons for field actions
        bfrm = ttk.Frame(right_frame)
        bfrm.pack(fill=tk.X, padx=6, pady=(0, 6))
        ttk.Button(bfrm, text="Add Field", command=self.add_field).pack(side=tk.LEFT)
        ttk.Button(bfrm, text="Update Field", command=self.update_field).pack(side=tk.LEFT, padx=6)
        ttk.Button(bfrm, text="Delete Field", command=self.delete_field).pack(side=tk.LEFT)
        ttk.Button(bfrm, text="Save Config", command=self.save).pack(side=tk.RIGHT)

    def _load(self):
        try:
            self.data = load_config(self.config_path)
        except Exception as e:
            messagebox.showerror("Error", f"Cannot load config: {e}")
            self.data = {"topics": [], "msg_defs": []}
        self.refresh_lists()

    def refresh_lists(self):
        # Topics
        self.topics_list.delete(0, tk.END)
        for i, t in enumerate(self.data.get("topics", [])):
            display = f"{i}: {t.get('topic_name','<no_name>')} [{t.get('msg_type','?')}]"
            self.topics_list.insert(tk.END, display)
        # MsgDefs
        self.msgdefs_list.delete(0, tk.END)
        for i, m in enumerate(self.data.get("msg_defs", [])):
            display = f"{i}: {m.get('msg_type','<no_type>')}"
            self.msgdefs_list.insert(tk.END, display)
        self.clear_editor()

    def clear_editor(self):
        self.header_label.config(text="Select a Topic or MsgDef to edit")
        for it in self.fields_tree.get_children():
            self.fields_tree.delete(it)
        self.selected_side = None
        self.selected_index = None
        self._clear_form()

    def _clear_form(self):
        self.field_kind.set("type")
        self.val_name_var.set("")
        self.type_var.set("")
        self.alt_var.set("")
        self.unit_var.set("")
        self.display_var.set("")
        self.range_min_var.set("")
        self.range_max_var.set("")
        self.mapping_var.set("")
        self.controls_var.set("")
        self.offset_var.set("")

    def on_topic_select(self, ev=None):
        sel = self.topics_list.curselection()
        if not sel:
            return
        idx = sel[0]
        self.selected_side = "topic"
        self.selected_index = idx
        t = self.data["topics"][idx]
        self.header_label.config(text=f"Topic: {t.get('topic_name')}  interval={t.get('interval')}")
        self.populate_fields(t.get("msg_fields", []))

    def on_msgdef_select(self, ev=None):
        sel = self.msgdefs_list.curselection()
        if not sel:
            return
        idx = sel[0]
        self.selected_side = "msgdef"
        self.selected_index = idx
        m = self.data["msg_defs"][idx]
        self.header_label.config(text=f"MsgDef: {m.get('msg_type')}")
        self.populate_fields(m.get("msg_fields", []))

    def populate_fields(self, fields):
        for it in self.fields_tree.get_children():
            self.fields_tree.delete(it)
        for i, f in enumerate(fields):
            kind = "msg_def" if "msg_def" in f else ("type" if "type" in f else "unknown")
            val_name = f.get("val_name", "")
            type_or_def = f.get("type") or f.get("msg_def") or ""
            self.fields_tree.insert("", "end", iid=str(i), values=(i, kind, val_name, type_or_def))
        self._clear_form()

    def on_field_select(self, ev=None):
        sel = self.fields_tree.selection()
        if not sel:
            return
        idx = int(sel[0])
        if self.selected_side is None:
            return
        item = self._get_current_container()[ "msg_fields" ][idx]
        # populate form
        if "msg_def" in item:
            self.field_kind.set("msg_def"); self.type_var.set(item.get("msg_def",""))
        else:
            self.field_kind.set("type"); self.type_var.set(item.get("type",""))
        self.val_name_var.set(item.get("val_name",""))
        self.alt_var.set(item.get("alt_name","") or "")
        self.unit_var.set(item.get("unit","") or "")
        self.display_var.set(item.get("display","") or "")
        rng = item.get("range")
        if isinstance(rng, list) and len(rng) >= 2:
            self.range_min_var.set(str(rng[0])); self.range_max_var.set(str(rng[1]))
        else:
            self.range_min_var.set("")
            self.range_max_var.set("")
        mapping = item.get("mapping")
        if isinstance(mapping, list):
            self.mapping_var.set(",".join(map(str, mapping)))
        else:
            self.mapping_var.set("")
        self.controls_var.set(item.get("controls","") or "")
        offset = item.get("offset")
        if isinstance(offset, list):
            self.offset_var.set(",".join(map(str, offset)))
        else:
            self.offset_var.set("")

    def _get_current_container(self):
        if self.selected_side == "topic":
            return self.data["topics"][self.selected_index]
        elif self.selected_side == "msgdef":
            return self.data["msg_defs"][self.selected_index]
        raise RuntimeError("No selection")

    def add_field(self):
        if self.selected_side is None:
            messagebox.showinfo("Select", "Select a topic or msgdef first")
            return
        container = self._get_current_container()
        if "msg_fields" not in container:
            container["msg_fields"] = []
        new = {"val_name": "new_field"}
        if self.field_kind.get() == "msg_def":
            new["msg_def"] = self.type_var.get() or "Unnamed"
        else:
            new["type"] = self.type_var.get() or "float32"
        container["msg_fields"].append(new)
        self.populate_fields(container["msg_fields"])
        # select new
        idx = len(container["msg_fields"]) - 1
        self.fields_tree.selection_set(str(idx))
        self.on_field_select()

    def update_field(self):
        sel = self.fields_tree.selection()
        if not sel:
            messagebox.showinfo("Select", "Select a field to update")
            return
        idx = int(sel[0])
        container = self._get_current_container()
        f = container["msg_fields"][idx]
        # set kind
        if self.field_kind.get() == "msg_def":
            # remove type if exists
            f.pop("type", None)
            f["msg_def"] = self.type_var.get().strip() or f.get("msg_def","")
        else:
            f.pop("msg_def", None)
            f["type"] = self.type_var.get().strip() or f.get("type","")
        f["val_name"] = self.val_name_var.get().strip() or f.get("val_name","")
        alt = self.alt_var.get().strip()
        if alt:
            f["alt_name"] = alt
        else:
            f.pop("alt_name", None)
        unit = self.unit_var.get().strip()
        if unit:
            f["unit"] = unit
        else:
            f.pop("unit", None)
        disp = self.display_var.get().strip()
        if disp:
            f["display"] = disp
        else:
            f.pop("display", None)
        # range parse
        rmin = self.range_min_var.get().strip()
        rmax = self.range_max_var.get().strip()
        if rmin != "" or rmax != "":
            try:
                # attempt numeric conversion, pick int if no decimal
                def conv(x):
                    return int(x) if x.isdigit() or (x and x.lstrip("-").isdigit()) else float(x)
                f["range"] = [conv(rmin) if rmin != "" else None, conv(rmax) if rmax != "" else None]
            except Exception:
                messagebox.showwarning("Range", "Range values should be numeric; saved as strings.")
                f["range"] = [rmin, rmax]
        else:
            f.pop("range", None)
        mapping = self.mapping_var.get().strip()
        if mapping:
            f["mapping"] = [m.strip() for m in mapping.split(",")]
        else:
            f.pop("mapping", None)
        controls = self.controls_var.get().strip()
        if controls:
            f["controls"] = controls
        else:
            f.pop("controls", None)
        offset = self.offset_var.get().strip()
        if offset:
            # try numeric list
            parts = [p.strip() for p in offset.split(",") if p.strip() != ""]
            try:
                f["offset"] = [int(p) if p.isdigit() or (p and p.lstrip("-").isdigit()) else float(p) for p in parts]
            except Exception:
                f["offset"] = parts
        else:
            f.pop("offset", None)

        # refresh view
        self.populate_fields(container.get("msg_fields", []))
        self.fields_tree.selection_set(str(idx))

    def delete_field(self):
        sel = self.fields_tree.selection()
        if not sel:
            messagebox.showinfo("Select", "Select a field to delete")
            return
        idx = int(sel[0])
        container = self._get_current_container()
        del container["msg_fields"][idx]
        self.populate_fields(container.get("msg_fields", []))

    def add_topic(self):
        name = simpledialog.askstring("New Topic", "Enter topic_name (e.g. tanking/new):")
        if not name:
            return
        msg_type = simpledialog.askstring("New Topic", "Enter msg_type (e.g. SomeMsg):") or ""
        interval = simpledialog.askinteger("New Topic", "Interval (ms):", initialvalue=1000)
        nid = max([t.get("id", 0) for t in self.data.get("topics", [])] + [0]) + 1
        new = {"id": nid, "topic_name": name, "msg_type": msg_type, "interval": interval or 1000, "msg_fields": []}
        self.data.setdefault("topics", []).append(new)
        self.refresh_lists()

    def delete_topic(self):
        sel = self.topics_list.curselection()
        if not sel:
            messagebox.showinfo("Select", "Select a topic to delete")
            return
        idx = sel[0]
        t = self.data["topics"][idx]
        if not messagebox.askyesno("Delete", f"Delete topic {t.get('topic_name')}?"):
            return
        del self.data["topics"][idx]
        self.refresh_lists()

    def add_msgdef(self):
        name = simpledialog.askstring("New MsgDef", "Enter msg_type (e.g. MyMsg):")
        if not name:
            return
        new = {"msg_type": name, "msg_fields": []}
        self.data.setdefault("msg_defs", []).append(new)
        self.refresh_lists()

    def delete_msgdef(self):
        sel = self.msgdefs_list.curselection()
        if not sel:
            messagebox.showinfo("Select", "Select a msgdef to delete")
            return
        idx = sel[0]
        m = self.data["msg_defs"][idx]
        if not messagebox.askyesno("Delete", f"Delete msg_def {m.get('msg_type')}?"):
            return
        del self.data["msg_defs"][idx]
        self.refresh_lists()

    def load_topic_from_file(self):
        path = filedialog.askopenfilename(title="Load JSON fragment for topic", filetypes=[("JSON", "*.json"), ("All", "*.*")])
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                fragment = json.load(f)
            # add as new topic (minimal acceptance)
            nid = max([t.get("id", 0) for t in self.data.get("topics", [])] + [0]) + 1
            fragment.setdefault("id", nid)
            self.data.setdefault("topics", []).append(fragment)
            self.refresh_lists()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load file: {e}")

    def save(self):
        try:
            b = save_config(self.data, self.config_path)
            messagebox.showinfo("Saved", f"Wrote {self.config_path}\nBackup created: {b}")
        except Exception as e:
            messagebox.showerror("Save error", str(e))


def main():
    app = ConfigTinker(ROOT_CONFIG)
    app.mainloop()


if __name__ == "__main__":
    main()