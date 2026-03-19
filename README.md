# Mission Control App
A web application responsible for displaying the current state of the rocket and the entire ground segment system.

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
    
3. Run the app (*Make sure to `chmod +x` the script!*):
    ```bash
    sudo chmod +x ./run.sh
    sudo chmod +x ./scripts/*.sh
    ```

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
If you don't see the folder **install** refer to *gs_interfaces* submodule README.

- #### (InfluxDB) <>:8086: connect: connection refused
    Make sure to set correct IP of your device (IP_ADDRESS) inside **.env** file.

- #### await import('source-map-support').then((r) => r.default.install()) <br> SyntaxError: Unexpected reserved word
    Fix:
    ```shell
    sudo npm cache clean -f
    sudo npm install -g n
    sudo n stable
    ```


## Knowledge Base
In a nutshell the app reads the config.json file, then dynamically creates instances of NodeHandler which are responsible for subscribing to single ROS topic, creating Websocket connection and inserting data to InfluxDB.

It's worth mentioning that messages related to rocket are send through *MAVLink protocol* and because of that they need to be parsed and copied into the `config.json` using `xml_to_json.py` script inside `mavlink` directory.

Single entry of the `config.json` looks like this:

```json
{
    "topic_name": "example/topic",
    "msg_type": "ExampleTopic",
    "msg_fields": [
        {
            "type": "std_msgs/Header",
            "val_name": "header"
        },
        {
            "type": "float32",
            "val_name": "temperature",
            "alt_name": "Tank Temperature",
            "unit": "°C",
            "display": "value",
            "range": [
                -10,
                20
            ],
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

`config.json` is just a wrapper for messages found inside **gs_interfaces** submodule. So any change in gs_interfaces should be reflected inside the config as well. You may ask: "So what's the point of keeping redundant definitions in both places?". The reason is that `config.json` allows us to dynamically populate the app with custom ranges, names and units for incoming data. In that way there is almost no need for hardcoded values inside server or frontend codebase. 
With that in mind, below are the available options (keys) you can use when adding a new message.

**topic_name** - <br>
**msg_type** - <br>
**interval** - <br>
**msg_fields** - # std_msgs/Header is required in every msg! <br>


### Project Structure

```text
gs_web_app/
├── backend
│   ├── main.js
│   ├── App.svelte
│   └── components/
│       └── NavBar.svelte
├── frontend
├── gs_interfaces
├── mavlink
├── scripts
├── services
├── shared
├── tests
├── .env
├── config.json
├── oled_display.py
├── run.sh
```