# Web app
It's simple app that we will work upon in creating bigger mission control system.


## Setup
1. Go to **app** directory and `npm install`
2. Create python environment `python3 -m venv venv`
3. Install required packages `pip install -r requirements.txt`
*(If you don't have ROS2 distro installed you won't be able to run the server)*

## Launching the app
0. Activate the environment `source venv/bin/activate`
1. Run example topic `python3 topics/publisher.py`
2. Run the server `python3 server/server.py`
3. Inside the **app** directory run the frontend `npm run dev`

~~Hope it works! - but I guess it's not working.~~


### Technologies used:

- [x] Svelte
- [x] FastApi
- [x] ROS2
- [ ] InfluxDB
- [ ] Docker
- [x] Python
