import json 
from os.path import expanduser, dirname, exists
from os import makedirs

CONFIG_FILENAME = expanduser("~") + "/.config/sumobot/sumobot_xbox_controller/" + "config.json"

# Config data read from the file
data = {}

config_keys = [
    "topic_name",
    "node_name",
    "polling_frequency",
    "use_controller"
]

default_config = {
    "topic_name" : "xbox_controller_data",
    "node_name" : "sumobot_xbox_controller",
    "polling_frequency" : 10,
    "use_controller" : True
}

def write_config():
    global data
    with open(CONFIG_FILENAME, 'w') as f:
        f.write(json.dumps(data, 
                           sort_keys=True, 
                           indent=4, 
                           separators=(',', ':')))

def load_config():
    global data
    try:
        with open(CONFIG_FILENAME, 'r') as f:
            data = json.loads(f.read())
    except IOError:
        pass
    # Check that every field exists
    for key in config_keys:
        if key not in data:
            create_new_file()
            return

def create_new_file():
    global data
    for key in config_keys:
        if key not in data:
            data[key] = default_config[key]
    write_config()

def init():
    # Create the config directory if it does not already exist
    if not exists(dirname(CONFIG_FILENAME)):
        try: 
            makedirs(dirname(CONFIG_FILENAME))
        except OSError as exc: # In case of race condition
            if exc.errno != errno.EEXIST:
                raise
    load_config()
