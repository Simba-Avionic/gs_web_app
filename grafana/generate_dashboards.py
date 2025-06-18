import os
import sys
import json
import shutil
import copy
import random
import gs_interfaces.msg

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from shared.paths import CONFIG_JSON_PATH


target_file_path = 'dashboards/dashboard.json'
backup_file_path = 'templates/dashboard_template.json'
shutil.copy(backup_file_path, target_file_path)
print(f"Backup created: {backup_file_path}")

with open(target_file_path, 'r') as file:
    target_data = json.load(file)

with open(CONFIG_JSON_PATH, 'r') as file:
    CONFIG = json.load(file)

with open('main_dashboard_config.json', 'r') as file:
    MAIN_DASHBOARD = json.load(file)

with open('templates/gauge_template.json', 'r') as file:
    GAUGE_TEMPLATE = json.load(file)

with open('templates/timeseries_template.json', 'r') as file:
    DEFAULT_TEMPLATE = json.load(file)

with open('templates/stat_template.json', 'r') as file:
    STAT_TEMPLATE = json.load(file)

target_data["id"] = random.randint(0, 1000)
idx = 0


def import_message_type(msg_type):
    return getattr(gs_interfaces.msg, msg_type)


def process_unit(unit: str):
    if unit == "°C":
        return "celsius"
    elif unit == "%":
        return "percent"
    elif unit == "m":
        return "lengthm"
    return unit


def find_topics_by_msg_type(msg_type):
    matching_topics = []
    for topic in CONFIG["topics"]:
        if topic["msg_type"] == msg_type:
            matching_topics.append(topic)
    return matching_topics


def find_field_in_topic(topic, field_name):
    for field in topic["msg_fields"]:
        if field["val_name"] == field_name:
            return field
    return None


def find_msg_def_by_type(msg_type):
    for msg_def in CONFIG.get("msg_defs", []):
        if msg_def["msg_type"] == msg_type:
            return msg_def
    return None


def handle_mapping(mapping):
    if not mapping:
        return []

    mappings = []

    if mapping == "open/closed":
        mappings = [
            {
                "options": {
                    "match": "false",
                    "result": {
                        "color": "red",
                        "index": 0,
                        "text": "CLOSED"
                    }
                },
                "type": "special"
            },
            {
                "options": {
                    "match": "true",
                    "result": {
                        "color": "green",
                        "index": 1,
                        "text": "OPEN"
                    }
                },
                "type": "special"
            }
        ]
    elif mapping == "on/off":
        mappings = [
            {
                "options": {
                    "match": "false",
                    "result": {
                        "color": "red",
                        "index": 0,
                        "text": "OFF"
                    }
                },
                "type": "special"
            },
            {
                "options": {
                    "match": "true",
                    "result": {
                        "color": "green",
                        "index": 1,
                        "text": "ON"
                    }
                },
                "type": "special"
            }
        ]
    elif mapping == "status":
        mappings = [
            {
                "options": {
                    "0": {
                        "color": "green",
                        "index": 0,
                        "text": "OK"
                    },
                    "1": {
                        "color": "red",
                        "index": 1,
                        "text": "NOT_DEFINED"
                    },
                    "2": {
                        "color": "red",
                        "index": 2,
                        "text": "ERROR"
                    },
                    "3": {
                        "color": "red",
                        "index": 3,
                        "text": "CONNECTION_ERROR"
                    },
                    "4": {
                        "color": "red",
                        "index": 4,
                        "text": "INITIALIZE_ERROR"
                    },
                    "5": {
                        "color": "red",
                        "index": 5,
                        "text": "BAD_VARIABLE_SIZE"
                    },
                    "6": {
                        "color": "red",
                        "index": 6,
                        "text": "INVALID_STATE"
                    }
                },
                "type": "value"
            },
            {
              "options": {
                "from": 7,
                "result": {
                  "index": 7,
                  "text": "UNKNOWN STATUS"
                },
                "to": 99999
              },
              "type": "range"
            }
        ]
    elif mapping == "state":
        mappings = [
            {
                "options": {
                    "1": {
                        "color": "",
                        "index": 0,
                        "text": "DISARMED"
                    },
                    "2": {
                        "color": "orange",
                        "index": 1,
                        "text": "ARMED"
                    },
                    "3": {
                        "color": "green",
                        "index": 2,
                        "text": "IGNITION"
                    },
                    "4": {
                        "color": "red",
                        "index": 3,
                        "text": "ABORTED"
                    }
                },
                "type": "value"
            },
            {
              "options": {
                "from": 5,
                "result": {
                  "index": 4,
                  "text": "UNKNOWN STATE"
                },
                "to": 99999
              },
              "type": "range"
            }
        ]
    elif mapping == "valve_pos":
        mappings= [
            {
              "options": {
                "from": 0,
                "result": {
                  "color": "red",
                  "index": 0,
                  "text": "CLOSED"
                },
                "to": 50
              },
              "type": "range"
            },
            {
              "options": {
                "from": 51,
                "result": {
                  "color": "green",
                  "index": 1,
                  "text": "OPEN"
                },
                "to": 100
              },
              "type": "range"
            }
        ]

    return mappings


def get_next_grid_position(panels, panel_width, panel_height):
    """
    Determines the next available position for a panel in the grid.
    Uses a first-fit approach to maintain a compact layout.

    Args:
        panels: List of existing panels
        panel_width: Width of the new panel
        panel_height: Height of the new panel

    Returns:
        Tuple of (x, y) coordinates for the new panel
    """
    if not panels:
        return (0, 0)

    dashboard_width= 24
    row_right_edges= {}

    for panel in panels:
        grid_pos= panel["gridPos"]
        panel_bottom= grid_pos["y"] + grid_pos["h"]

        for row in range(grid_pos["y"], panel_bottom):
            current_edge= row_right_edges.get(row, 0)
            new_edge= grid_pos["x"] + grid_pos["w"]
            row_right_edges[row]= max(current_edge, new_edge)

    available_positions= []

    max_row= max(row_right_edges.keys()) + 1 if row_right_edges else 0

    for row in range(0, max_row + 1):
        right_edge= row_right_edges.get(row, 0)

        if right_edge + panel_width <= dashboard_width:
            has_space= True
            for r in range(row, row + panel_height):
                if row_right_edges.get(r, 0) > right_edge:
                    has_space= False
                    break

            if has_space:
                available_positions.append((right_edge, row))

    if available_positions:
        # Sort by y coordinate
        available_positions.sort(key=lambda pos: pos[1])
        return available_positions[0]

    return (0, max_row + 1)

def populate_field(dashboard_field, msg_type):
    global idx
    field_name= dashboard_field['val_name']

    if field_name == "header":
        return

    matching_topics= find_topics_by_msg_type(msg_type)
    if not matching_topics:
        print(
            f"Warning: No topics found in config.json for message type {msg_type}")
        return

    for config_topic in matching_topics:

        config_field= find_field_in_topic(config_topic, field_name)
        if not config_field:
            continue

        if "gs_interfaces/" in config_field['type']:
            nested_type= config_field['type'].split('/')[1]
            nested_msg_def= find_msg_def_by_type(nested_type)

            if nested_msg_def:
                for nested_field in nested_msg_def["msg_fields"]:
                    nested_name= f"{field_name}_{nested_field['val_name']}"
                    nested_dashboard_field= {'val_name': nested_name}

                    if 'stat' in dashboard_field:
                        nested_dashboard_field['stat']= dashboard_field['stat']

                    # Process this nested field (might need to be added to CONFIG first)
                    # For now, we'll just print a message
                    print(
                        f"Found nested field: {nested_name} from {nested_type}")
            continue

        if 'stat' in dashboard_field:
            template_tmp= copy.deepcopy(STAT_TEMPLATE)
            template_tmp['fieldConfig']['defaults']['mappings']= handle_mapping(dashboard_field['stat']['mapping'])
            template_tmp['options']['reduceOptions']['fields']= f"/^{field_name} {msg_type}$/"
            panel_width= 4  # Stats are narrower
            panel_height = 3
        else:
            template_tmp= copy.deepcopy(STAT_TEMPLATE)
            # template_tmp['fieldConfig']['defaults']['custom']['axisLabel']= field_name
            template_tmp['fieldConfig']['defaults']['color']['mode']= "palette-classic-by-name"
            template_tmp['options']['percentChangeColorMode'] = "same_as_value"
            template_tmp['options']['showPercentChange'] = True
            template_tmp['options']['colorMode'] = "value"
            template_tmp['options']['graphMode'] = "area"

            panel_width= 4
            panel_height = 6

        if 'unit' in config_field:
            template_tmp['fieldConfig']['defaults']['unit']= process_unit(
                config_field['unit'])

        template_tmp['description']= msg_type
        template_tmp['title'] = field_name
        template_tmp['id'] = idx

        query = (
            f'from(bucket: "simba_bucket")\n'
            f'  |> range(start: v.timeRangeStart, stop: v.timeRangeStop)\n'
            f'  |> filter(fn: (r) => r["_measurement"] == "{msg_type}")\n'
            f'  |> filter(fn: (r) => r["_field"] == "{field_name}")\n'
            f'  |> aggregateWindow(every: v.windowPeriod, fn: last, createEmpty: false)\n'
            f'  |> yield(name: "last")'
        )
        template_tmp['targets'][0]['query'] = query

        next_pos = get_next_grid_position(target_data["panels"], panel_width, panel_height)
        template_tmp['gridPos']['x'] = next_pos[0]
        template_tmp['gridPos']['y'] = next_pos[1]
        template_tmp['gridPos']['w'] = panel_width
        template_tmp['gridPos']['h'] = panel_height
        idx += 1

        target_data["panels"].append(template_tmp)

def get_all_fields():
    """
    Collect and sort all fields from the main dashboard configuration.
    Returns fields with 'stat' option first, maintaining original ordering within groups.
    """
    stat_fields = []
    regular_fields = []
    
    for dashboard_msg in MAIN_DASHBOARD["topics"]:
        msg_type = dashboard_msg["msg_type"]
        if "msg_fields" not in dashboard_msg:
            continue
            
        for dashboard_field in dashboard_msg['msg_fields']:
            if "val_name" not in dashboard_field:
                continue
                
            if 'stat' in dashboard_field:
                stat_fields.append((dashboard_field, msg_type))
            else:
                regular_fields.append((dashboard_field, msg_type))
    
    return stat_fields + regular_fields

all_fields = get_all_fields()
for dashboard_field, msg_type in all_fields:
    populate_field(dashboard_field, msg_type)

with open(target_file_path, 'w') as file:
    json.dump(target_data, file, indent=4)

print("Dashboard created successfully.")