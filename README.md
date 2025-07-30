# Mission Control App
Responsible for displaying current state of a rocket and the whole ground segment system. <br>
In a nutshell it dynamically creates instances of NodeHandler which are responsible for subscribing to single ROS topic,
through websocket it sends data to frontend and finally creates thread to insert received data to InfluxDB.

Those Node Handlers are populated based on `config.json` file.
It's worth mentioning that messages related to rocket are send through *MAVLink protocol* and because of that they need to be parsed and copied into the `config.json` using `xml_to_json.py` script inside `mavlink` directory
Single entry of the `config.json` looks like this:

```json
{
    "topic_name": "example/topic",
    "msg_type": "ExampleTopic", # Name of .msg file
    "interval": 1000,
    "place": "gs", # from where the topic is coming from (possible values: "gs", "rocket") 
    "msg_fields": [ # std_msgs/Header is required in every msg type!
        {
            "type": "std_msgs/Header",
            "val_name": "header"
        },
        {
            "type": "float32",
            "val_name": "temperature",
            "unit": "°C"
        },
        {
            "type": "int32",
            "val_name": "load_cell",
            "unit": "kg"
        },
        {
            "type": "bool",
            "val_name": "is_alive",
            "unit": "bool"
        }
    ]
}
```

## Prerequisites
- Ubuntu 22.04
- Python 3.10 (Should be already available in Ubuntu 22.04)
- Git ( `sudo apt install git-all` ) 

## Setup
1. Clone this repo and its submodules 

    ```
    git clone --recursive https://github.com/Simba-Avionic/gs_web_app
    ```
2. Run the `./scripts/install.sh`. It will try to install *Docker, ROS2, npm* and some other related packages. After completion restart the system.
    If script fails, try to install ROS2 and/or Docker manually:
    - ROS2 Humble (https://docs.ros.org/en/humble/Installation.html)
    - Docker (https://docs.docker.com/engine/install/)
    App was only tested with Python3.10 so make sure you have it installed.

## Launching the app (step by step)
1. Activate the Python environment (you can also add to ~/.bashrc)

    ```bash
    source venv/bin/activate
    ```
    
2. Make sure to source ROS environment (you can also add to ~/.bashrc)
   
    ```bash
    source build/install/setup.bash
    ```
    
3. Run the app (*Make sure to `chmod` the script!*):
    ```bash
    ./run.sh run_all
    ```

    You can also check available commands:
    ```bash
    ./run.sh help
    ```

~~Hope it works! - but I guess it doesn't.~~

## Common errors

- #### ModuleNotFoundError: No module named 'gs_interfaces'
    You need to source the environment from the main directory: `source install/setup.bash`.
If you don't see the folder **install** refer to *gs_interfaces* README.

- #### (Grafana & InfluxDB) <>:8086: connect: connection refused
    Make sure to set correct IP of your device (IP_ADDRESS) inside **.env** file.

- #### await import('source-map-support').then((r) => r.default.install()) <br> SyntaxError: Unexpected reserved word
    Fix:
    ```shell
    sudo npm cache clean -f
    sudo npm install -g n
    sudo n stable
    ```
### Technologies used:

- [x] Svelte
- [x] FastApi
- [x] ROS2
- [x] InfluxDB
- [x] Docker
- [x] Python
- [x] Grafana
- [x] MAVLink
