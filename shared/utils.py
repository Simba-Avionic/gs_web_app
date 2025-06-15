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


def patch_mavlink_dialect(dialect_module=None):
    """
    Patch a MAVLink dialect module with required standard enum values
    that mavutil expects to find.
    
    Args:
        dialect_name: The name of the dialect to patch (default: "simba")
    
    Returns:
        The patched dialect module
    """
    
    try:
        import pymavlink.dialects.v10.simba as dialect_module
        print("Imported dialect from standard location")
    except ImportError as e:
        print(f"Failed to import pymavlink.dialects.v10.simba: {e}")

    if dialect_module:
        dialect_module.MAV_TYPE_GENERIC = 0
        dialect_module.MAV_TYPE_FIXED_WING = 1
        dialect_module.MAV_TYPE_QUADROTOR = 2
        dialect_module.MAV_TYPE_COAXIAL = 3
        dialect_module.MAV_TYPE_HELICOPTER = 4
        dialect_module.MAV_TYPE_ANTENNA_TRACKER = 5
        dialect_module.MAV_TYPE_GCS = 6
        dialect_module.MAV_TYPE_AIRSHIP = 7
        dialect_module.MAV_TYPE_FREE_BALLOON = 8
        dialect_module.MAV_TYPE_ROCKET = 9
        dialect_module.MAV_TYPE_GROUND_ROVER = 10
        dialect_module.MAV_TYPE_SURFACE_BOAT = 11
        dialect_module.MAV_TYPE_SUBMARINE = 12
        dialect_module.MAV_TYPE_HEXAROTOR = 13
        dialect_module.MAV_TYPE_OCTOROTOR = 14
        dialect_module.MAV_TYPE_TRICOPTER = 15
        dialect_module.MAV_TYPE_VTOL_DUOROTOR = 19
        dialect_module.MAV_TYPE_VTOL_QUADROTOR = 20
        dialect_module.MAV_TYPE_VTOL_TILTROTOR = 21
        dialect_module.MAV_TYPE_DODECAROTOR = 25
        dialect_module.MAV_TYPE_DECAROTOR = 26
        
        dialect_module.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
        dialect_module.MAV_MODE_FLAG_STABILIZE_ENABLED = 4
        dialect_module.MAV_MODE_FLAG_GUIDED_ENABLED = 8
        dialect_module.MAV_MODE_FLAG_AUTO_ENABLED = 16
        dialect_module.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64
        dialect_module.MAV_MODE_FLAG_SAFETY_ARMED = 128
        
        return dialect_module