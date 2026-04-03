#!/bin/bash
set -e

# Source the main ROS 2 installation
source /opt/ros/humble/setup.bash

# Source your custom built messages
if [ -f "/app/build/install/setup.bash" ]; then
    source /app/build/install/setup.bash
fi

# Execute the command passed into the container
exec "$@"