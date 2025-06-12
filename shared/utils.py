# Mapping from MAVLink types to ROS message types
TYPE_MAPPINGS = {
    'uint64_t': 'uint64',
    'int64_t': 'int64',
    'uint32_t': 'uint32',
    'int32_t': 'int32',
    'uint16_t': 'uint16',
    'int16_t': 'int16',
    'uint8_t': 'uint8',
    'int8_t': 'int8',
    'float': 'float32',
    'double': 'float64'
}

# Convert MAVLink type to ROS message type
def get_type_mapping(mavlink_type):
    if mavlink_type in TYPE_MAPPINGS:
        return TYPE_MAPPINGS.get(mavlink_type)
    else:
        raise ValueError(f"Unknown MAVLink type: {mavlink_type}")

# Convert message name from UPPERCASE_UNDERSCORE to CamelCase
def convert_message_name(name):
    return ''.join(word.capitalize() for word in name.lower().split('_'))