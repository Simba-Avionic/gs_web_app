import json
import pathlib
 
def get_node_name(message_type: str):
    # this function assumes that every node has its own message type
    # Read config.json
    with open(get_config_path(), 'r') as f:
        config = json.load(f)    
    
    for topic in config['topics']:
        if topic['msg_type'] == message_type:
            return topic['topic_name']
        
    raise FileNotFoundError(f"Topic of type {message_type} not found")


def get_config_path():
    # Get the current file path
    current_file = pathlib.Path(__file__).resolve()

    # Traverse up to the 'gs_web_app' directory
    searched_dir = current_file
    i = 0 
    while searched_dir.name != 'gs_web_app'  and i < 10: # search iter limit
        searched_dir = searched_dir.parent
        i += 1

    # Check if we found the 'gs_web_app' directory
    if searched_dir.name != 'gs_web_app':
        raise FileNotFoundError("The directory 'gs_web_app' was not found in the path.")

    return searched_dir / 'config.json'

