# Mission Control App
Responsible for displaying current state of a rocket and the whole ground segment system.


## Setup
0. Install ROS2 distro e.g. Humble (https://docs.ros.org/en/humble/Installation.html)
1. Create python virtual environment `python3 -m venv venv`
2. Install required packages `pip install -r requirements.txt`
3. Go to **app** directory and run `npm install`
*(If you don't have ROS2 distro installed you won't be able to run the server)*

## Launching the app
0. Activate the environment `source venv/bin/activate`
1. Run the server `python3 server/server.py`
2. Inside the **app** directory run the frontend `npm run dev`
3. (Optional) Run example topic `python3 topics/publisher.py`

~~Hope it works! - but I guess it's not.~~


### Technologies used:

- [x] Svelte
- [x] FastApi
- [x] ROS2
- [x] InfluxDB
- [x] Docker
- [x] Python
