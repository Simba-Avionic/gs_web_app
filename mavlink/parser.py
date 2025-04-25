import xml.etree.ElementTree as ET

def parse_simba_xml(path_to_xml):
    """Parse the simba.xml file and return a list of message definitions."""
    tree = ET.parse(path_to_xml)
    root = tree.getroot()

    messages = []
    for message in root.findall('./messages/message'):
        msg_name = message.get('name')
        fields = []

        for field in message.findall('./field'):
            field_type = field.get('type')
            field_name = field.get('name')
            fields.append({
                "type": field_type,
                "name": field_name
            })

        messages.append({
            "name": msg_name,
            "fields": fields
        })

    return messages

if __name__ == "__main__":
    print(parse_simba_xml("message_definitions/v1.0/simba.xml"))