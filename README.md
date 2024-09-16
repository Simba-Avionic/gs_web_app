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
    "interval": 1000, # How often will the NodeHandler try to obtain new msg (in ms)
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
- ROS2 distro e.g. Humble (https://docs.ros.org/en/humble/Installation.html)
- Docker (https://docs.docker.com/engine/install/)

## Setup
0. Create python virtual environment `python3 -m venv venv`
1. Install required packages `pip install -r requirements.txt`
2. Build ROS messages package `colcon build --packages-select gs_interfaces`
3. Go to **app** directory and run `npm install`<br>

## Launching the app (step by step)
0. Activate the Python environment `source venv/bin/activate`
1. Make sure to source ROS environment `source install/setup.bash`
2. Run the database `cd server/database && docker compose up -d`
3. Run the server `python3 server/server.py` or `cd server && uvicorn main:app`
4. Inside the **app** directory run the frontend `npm run dev`
5. (Optional) Run the Grafana visualisation `cd grafana && python generate_dashboards.py && docker compose up -d`
6. (Optional) Run example topics `./sim_nodes/run_all_sim_nodes.sh`

~~Hope it works! - but I guess it doesn't.~~

## Common errors

### ModuleNotFoundError: No module named 'gs_interfaces'
You need to source the environment from the main directory: `source install/setup.bash`.
If you don't see the folder **install** refer to *gs_interfaces* README.

### (Grafana & InfluxDB) <>:8086: connect: connection refused
Make sure to set correct IP of your device (IP_ADDRESS) inside **.env** file.

### Technologies used:

- [x] Svelte
- [x] FastApi
- [x] ROS2
- [x] InfluxDB
- [x] Docker
- [x] Python
- [x] Grafana
