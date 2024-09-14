import json
import shutil
import copy

def process_unit(unit : str):
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
backup_file_path = 'dashboards/dashboard_template.json'
shutil.copy(backup_file_path, target_file_path)
print(f"Backup created: {backup_file_path}")

with open(target_file_path, 'r') as file:
    target_data = json.load(file)


with open(config_file_path, 'r') as file:
    CONFIG = json.load(file)

idx = 0
for msg in CONFIG["topics"]:
    for msg_field in msg['msg_fields']:
        if msg_field['val_name'] != "header":
            template_tmp = copy.deepcopy(TEMPLATE)
            template_tmp['description'] = msg['msg_type']
            template_tmp['fieldConfig']['defaults']['custom']['axisLabel'] = msg_field['val_name']
            template_tmp['fieldConfig']['defaults']['unit'] = process_unit(msg_field['unit'])
            template_tmp['title'] = f"{msg['topic_name']}/{msg_field['val_name']}"
            template_tmp['id'] = idx
            template_tmp['targets'][0]['query'] = f'from(bucket: "simba_bucket")\r\n  |> range(start: v.timeRangeStart, stop: v.timeRangeStop)\r\n  |> filter(fn: (r) => r["_measurement"] == "{msg["msg_type"]}")\r\n  |> filter(fn: (r) => r["_field"] == "{msg_field["val_name"]}")\r\n  |> aggregateWindow(every: v.windowPeriod, fn: last, createEmpty: false)\r\n  |> yield(name: "last")'
            target_data["panels"].append(template_tmp)
            template_tmp['gridPos']['x'] = (idx % 4) * 6
            template_tmp['gridPos']['y'] = int(idx / 4)
            idx +=1


# from(bucket: \"simba_bucket\")\r\n  |> range(start: v.timeRangeStart, stop: v.timeRangeStop)\r\n  |> filter(fn: (r) => r[\"_measurement\"] == \"SIMBA_TEMPERATURE_STATUS\")\r\n  |> filter(fn: (r) => r[\"_field\"] == \"t_1\" or r[\"_field\"] == \"t_2\" or r[\"_field\"] == \"t_3\" or r[\"_field\"] == \"t_4\")\r\n  |> aggregateWindow(every: v.windowPeriod, fn: mean, createEmpty: false)\r\n  |> yield(name: \"mean\")",

# from(bucket: "simba_bucket")
#   |> range(start: v.timeRangeStart, stop: v.timeRangeStop)
#   |> filter(fn: (r) => r["_measurement"] == "LoadCells")
#   |> filter(fn: (r) => r["_field"] == "load_cell_1")
#   |> aggregateWindow(every: v.windowPeriod, fn: last, createEmpty: false)
#   |> yield(name: "last")

with open(target_file_path, 'w') as file:
    json.dump(target_data, file, indent=4)

print("Dashboard created successfully.")