# Mission Control App
Responsible for displaying current state of a rocket and the whole ground segment system. <br>
In a nutshell it dynamically creates instances of NodeHandler which are responsible for subscribing to single ROS topic,
creating websocket to send data to frontend and finally creating thread to insert received data to InfluxDB.

This NodeHandlers are populated based on **config.json** file.
Single entry looks like this:

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
- ROS2 distro e.g. Humble (https://docs.ros.org/en/humble/Installation.html)
- Docker (https://docs.docker.com/engine/install/)

## Setup
0. Clone this repo and its submodules `git clone --recursive https://github.com/Simba-Avionic/gs_web_app`
1. Run the `install.sh` script.
App was only tested with Python3.10 so make sure you have it installed.

## Launching the app (step by step)
0. Activate the Python environment

    ```bash
    source venv/bin/activate
    ```
    
1. Make sure to source ROS environment
   
    ```bash
    source install/setup.bash
    ```
    
2. Run the database
   
   ```bash
   cd server/database
   docker compose --env-file <path_to_env_file> up -d
   ```
   
3. Run the server
   
   ```bash
   cd server
   python3 main.py
   ```
   
4. Run the frontend
   
    ```bash
    cd app
    npm run dev
    ```
    
5. Run the Grafana visualisation
   
    ```bash
    cd grafana
    python generate_dashboards.py
    docker compose up -d
    ```
    
6. (Optional) Run example topics
    ```bash
    ./sim_nodes/run_all_sim_nodes.sh
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
