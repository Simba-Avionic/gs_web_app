# ROS2 Knowledge Base

An overview of working with ROS2, assuming ROS2 is properly set up on your system.

## In a Nutshell
- The typical workflow for running already created nodes is encapsulated in the `run_all_sim_nodes.sh` script.
- To run all simulation nodes simultaneously, execute `./run_all_sim_nodes.sh`.
- The `gs_web_app` directory is treated as a ROS2 workspace. As of 05/04, we don't maintain a separate `/src` folder for packages.

## Typical Package/Nodes Development Workflow
1. **Create a Package**:
   - Use `ros2 pkg create --build-type ament_python <package_name> --dependencies rclpy std_msgs` for Python nodes (`ament_cmake` for C++).
   - **Package Structure**: Key directories include:
     - `package.xml`: Package metadata and dependencies.
     - `setup.py`: Installation script.
     - `resource/<package_name>`: Contains an index marker file.

2. **Nodes Development Pattern**:
   - Write a Python script enabling `rclpy` to create a ROS node. Files ending with *pub* typically follow a common structure, whereas files without *pub* might include both publisher and subscriber functionalities.
   - Edit `setup.py` to make your node executable by adding to the **entry_points** section: `"<executable name> = <package name>.<python file name>:main"`.
     - Example: `sim_valve_sensors_pub = sim_nodes.sim_valve_sensors_pub:main`.
     - note that executable name and python file name might be the same

3. **Build the Package**:
   - `colcon build` to build all packages in the workspace or `colcon build --packages-select <package_name>` for specific ones.
   - you can also use `colcon build --symlink-install` that ensures that package is build before every run
4. **Source the Setup File**:
   - Run `source install/setup.bash` to make the packages available.
5. **Run the Node**:
   - Execute `ros2 run <package_name> <node_name>` to test your node.

## Useful Terminal Commands for Nodes
- `rqt_graph` visualizes nodes and topic connections.
- `ros2 run <package name> <node executable name>` runs a built node.
- `ros2 node list`, `ros2 topic list` show running nodes and topics.
- `ros2 topic echo <topic>` displays messages from a topic.
- `ros2 topic pub <topic name> <message path> <message in JSON format>` publishes a message repeatedly.
    - e.g. `ros2 topic pub -1 /rocket/telemetry my_rocket_pkg/msg/RocketTelemetry "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: '0'}, altitude: 100.0, velocity: 50.0, pressure: 1013.25, acceleration: 9.8, temperature: 15.0}"` (-1 is for publishing message once)

## Python Aspects of Working with ROS
- For rapid development, Python nodes can be run directly with `python3 <python_file_name.py>` after sourcing the ROS2 environment, skipping the `colcon build` step. Ensure your script is executable with `chmod +x my_node.py` and then run it with `./my_node.py`.

## Common Errors
- **No module named 'gs_interfaces.msg'**: Indicates the custom message package is not found. Make sure the package is built and the workspace is sourced.
- **command not found: ros2**: Ensure ROS2 is installed and sourced correctly. For convenience, add the source command to your `~/.bashrc` file.
