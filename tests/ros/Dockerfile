FROM ros:humble

# copying ros2 messages
RUN mkdir -p /simba_ws/src
COPY gs_interfaces /simba_ws/src/gs_interfaces

# building ros2 messages
WORKDIR /simba_ws
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select gs_interfaces

# installing python
RUN apt-get update && apt-get install -y python3-pip

# copying actual tests
COPY tests/ros ./tests
COPY config.json ./tests
COPY shared ./tests/shared

# start
CMD bash -c "\
source /opt/ros/humble/setup.bash && \
source /simba_ws/install/setup.bash && \
python3 /simba_ws/tests/dynamic_publisher_sim.py"