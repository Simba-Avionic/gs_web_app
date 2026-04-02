import xml.etree.ElementTree as ET
import json
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import shared.utils as utils
from shared.paths import SIMBA_XML_PATH

output_json = "temp.json"  # Output JSON file

def convert_xml_to_json(SIMBA_XML_PATH, output_json):
    tree = ET.parse(SIMBA_XML_PATH)
    root = tree.getroot()

    config = {"topics": []}

    for message in root.findall(".//message"):
        topic_name = f"mavlink/{message.get('name').lower()}"
        if "_cmd" in topic_name:
            continue  # Skip command messages

        topic = {
            "id": int(message.get("id")),
            "topic_name": topic_name,
            "msg_type": utils.convert_message_name(message.get("name")),
            "place": "rocket",
            "msg_fields": [
                {
                    "type": "std_msgs/Header",
                    "val_name": "header"
                }
            ]
        }

        for field in message.findall("field"):
            field_entry = {
                "type": utils.get_type_mapping(field.get("type")),
                "val_name": field.get("name")
            }

            field_name = field.get("name").lower()
            if 'temp' in field_name:
                field_entry["unit"] = "°C"
            elif 'pressure' in field_name:
                field_entry["unit"] = "bar"
            elif 'alt' in field_name:
                field_entry["unit"] = "m"
            
            
            topic["msg_fields"].append(field_entry)

        config["topics"].append(topic)

    with open(output_json, "w") as json_file:
        json.dump(config, json_file, indent=4)

    print(f"Converted {SIMBA_XML_PATH} to {output_json}")

def extract_enums(xml_path):
    """Extract all enums from the XML file"""
    tree = ET.parse(xml_path)
    root = tree.getroot()
    
    enums = {}
    
    for enum in root.findall(".//enum"):
        enum_name = enum.get("name")
        is_bitmask = enum.get("bitmask") == "true"
        
        enum_values = {}
        for entry in enum.findall("entry"):
            value = int(entry.get("value"))
            name = entry.get("name")
            enum_values[value] = name
        
        enums[enum_name] = {
            "values": enum_values,
            "is_bitmask": is_bitmask
        }
    
    return enums

if __name__ == "__main__":
    convert_xml_to_json(SIMBA_XML_PATH, output_json)