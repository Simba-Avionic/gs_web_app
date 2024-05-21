# Assumes that submodule gs_interfaces is present
cd "$(dirname "$0")"
cd ..
colcon build  # --cmake-clean-cache # sometimes help if changing script doesn't seem to do anything

PACKAGE_NAME="sim_nodes"
# List of nodes (executable names)
NODES=("sim_valve_sensors_pub" "sim_valve_servos_pub" "sim_load_cells" "sim_mr_general_publisher" "sim_radio_433_pub" "sim_rocket_telemetry_pub" "sim_rocket_status")



echo "Sourcing ROS environment..."
source /opt/ros/humble/setup.bash  # you might have to change it if different path

echo "Sourcing your custom workspace environment..."
source install/setup.bash


for NODE in "${NODES[@]}"; do
    ros2 run $PACKAGE_NAME $NODE &
done

wait