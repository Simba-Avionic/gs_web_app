import json
import shutil
import copy
import random
import gs_interfaces.msg


def process_unit(unit: str):
    if unit == "°C":
        return "celsius"
    elif unit == "%":
        return "percent"
    elif unit == "m":
        return "lengthm"
    return unit


TEMPLATE = {
      "datasource": {
        "default": True,
        "type": "influxdb",
        "uid": "P5697886F9CA74929"
      },
      "fieldConfig": {
        "defaults": {
          "color": {
            "mode": "palette-classic-by-name"
          },
          "custom": {
            "axisBorderShow": False,
            "axisCenteredZero": False,
            "axisColorMode": "text",
            "axisLabel": "",
            "axisPlacement": "auto",
            "barAlignment": 0,
            "barWidthFactor": 0.6,
            "drawStyle": "line",
            "fillOpacity": 0,
            "gradientMode": "none",
            "hideFrom": {
              "legend": False,
              "tooltip": False,
              "viz": False
            },
            "insertNulls": False,
            "lineInterpolation": "linear",
            "lineWidth": 1,
            "pointSize": 5,
            "scaleDistribution": {
              "type": "linear"
            },
            "showPoints": "auto",
            "spanNulls": False,
            "stacking": {
              "group": "A",
              "mode": "none"
            },
            "thresholdsStyle": {
              "mode": "off"
            }
          },
          "mappings": [],
          "thresholds": {
            "mode": "absolute",
            "steps": [
              {
                "color": "green",
                "value": None
              }
            ]
          }
        },
        "overrides": []
      },
      "gridPos": {
        "h": 6,
        "w": 6,
        "x": 0,
        "y": 0
      },
      "id": 1,
      "options": {
        "legend": {
          "calcs": [],
          "displayMode": "list",
          "placement": "bottom",
          "showLegend": True
        },
        "tooltip": {
          "mode": "single",
          "sort": "none"
        }
      },
      "targets": [
        {
          "datasource": {
            "type": "influxdb",
            "uid": "P5697886F9CA74929"
          },
          "query": "",
          "refId": "A"
        }
      ],
      "title": "",
      "type": "timeseries"
}

config_file_path = '../config.json'
target_file_path = 'dashboards/dashboard.json'
backup_file_path = 'dashboard_template.json'
shutil.copy(backup_file_path, target_file_path)
print(f"Backup created: {backup_file_path}")

with open(target_file_path, 'r') as file:
    target_data = json.load(file)


with open(config_file_path, 'r') as file:
    CONFIG = json.load(file)

target_data["id"] = random.randint(0, 1000)
idx = 0

def import_message_type(msg_type):
    return getattr(gs_interfaces.msg, msg_type)

def populate_field(msg_field):
	global idx
	if "gs_interfaces/" in msg_field['type']:
		package_name, message_name = msg_field['type'].split('/')
		nested_msg_type = import_message_type(message_name)
		nested_msg = nested_msg_type()

		if hasattr(nested_msg, '__slots__'):
			data = nested_msg_type.get_fields_and_field_types()
			msg_fields = [
				{'val_name': f"{msg_field['val_name']}_{key}", 'type': value} for key, value in data.items()
			]
		for msg_field in msg_fields:
			populate_field(msg_field)

	if msg_field['val_name'] != "header":
		template_tmp = copy.deepcopy(TEMPLATE)
		template_tmp['description'] = msg['msg_type']
		template_tmp['fieldConfig']['defaults']['custom']['axisLabel'] = msg_field['val_name']
		if 'unit' in msg_field:
			template_tmp['fieldConfig']['defaults']['unit'] = process_unit(msg_field['unit'])
		template_tmp['title'] = f"{msg['topic_name']}/{msg_field['val_name']}"
		template_tmp['id'] = f"{msg['topic_name']}/{msg_field['val_name']}"
		
		query = (
			f'from(bucket: "simba_bucket")\n'
			f'  |> range(start: v.timeRangeStart, stop: v.timeRangeStop)\n'
			f'  |> filter(fn: (r) => r["_measurement"] == "{msg["msg_type"]}")\n'
			f'  |> filter(fn: (r) => r["_field"] == "{msg_field["val_name"]}")\n'
			f'  |> aggregateWindow(every: v.windowPeriod, fn: last, createEmpty: false)\n'
			f'  |> yield(name: "last")'
		)
		template_tmp['targets'][0]['query'] = query

		target_data["panels"].append(template_tmp)
		template_tmp['gridPos']['x'] = (idx % 4) * 6
		template_tmp['gridPos']['y'] = int(idx / 4)
		idx += 1

for msg in CONFIG["topics"]:
	for msg_field in msg['msg_fields']:
		populate_field(msg_field)

with open(target_file_path, 'w') as file:
    json.dump(target_data, file, indent=4)

print("Dashboard created successfully.")