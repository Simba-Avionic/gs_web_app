#!/bin/bash

# You can adjust the APP_PATH if app is installed in a different location
APP_PATH="$HOME"
# echo "$APP_PATH"
BASHRC="$HOME/.bashrc"
LINE="source $APP_PATH/gs_web_app/build/install/setup.bash"

function source_ros() {
    source $APP_PATH/gs_web_app/build/install/setup.bash
    if [ $? -ne 0 ]; then
        echo "Failed to source ROS 2 environment."
        exit 1
    fi
}

function build_ros_msgs() {
    echo "Building ROS 2 messages..."
    cd gs_interfaces || exit
    python3 generate_ros2_messages.py
    if [ $? -ne 0 ]; then
        echo "Failed to generate ROS 2 messages."
        exit 1
    fi

    cd ..

    colcon --log-base build/log build --packages-select gs_interfaces --build-base build/build --install-base build/install
    if [ $? -ne 0 ]; then
        echo "Failed to build gs_interfaces package."
        exit 1
    fi

    # Check if the line in .bashrc already exists
    if grep -Fxq "$LINE" "$BASHRC"; then
        echo "Source line already exists in .bashrc"
    else
        echo "$LINE" >> "$BASHRC"
    fi

    echo "ROS 2 messages built & sourced successfully."
}

function build_mavlink_msgs() {
    echo "Generating MAVLink definitions using setup.sh..."
    cd mavlink|| exit
    chmod +x setup.sh
    ./setup.sh simba simba_mavlink/simba.xml
    if [ $? -ne 0 ]; then
        echo "Failed to generate MAVLink definitions."
        exit 1
    fi
    cd ..
    echo "MAVLink definitions generated successfully."
}

function publish_test_ros_msgs() {
    echo "Running custom test messages (python3 tests/ros/run.py)..."
    python3 tests/ros/run.py
}

function run_mavlink_client() {
    echo "Starting the MAVLink client..."
}

function run_database() {
    echo "Starting the database (docker-compose up -d)..."
    sudo docker compose --env-file .env up -d influxdb
    if [ $? -ne 0 ]; then
        echo "Failed to start the database."
        exit 1
    fi
}

function run_grafana() {
    cd grafana || exit
    echo "Generating Grafana dashboards..."
    python3 generate_dashboards.py

    cd ..
    
    echo "Starting Grafana (docker-compose up -d)..."
    sudo docker compose --env-file .env up -d grafana
    if [ $? -ne 0 ]; then
        echo "Failed to start the grafana."
        exit 1
    fi
    
    cd ..
}

function run_docker_stack() {

    if [ "$1" = "--clean" ]; then
        echo "Cleaning up Docker containers first..."
        chmod +x ./scripts/cleanup_docker.sh
        ./scripts/cleanup_docker.sh grafana
        ./scripts/cleanup_docker.sh influxdb
        if [ $? -ne 0 ]; then
            echo "Docker cleanup failed."
            exit 1
        fi
    fi

    echo "Starting InfluxDB + Grafana stack ..."
    sudo docker compose --env-file .env up -d
    if [ $? -ne 0 ]; then
        echo "Failed to start the Docker stack."
        exit 1
    fi
}

function run_app() {
    echo "Starting the app (npm run dev)..."
    cd frontend || exit
    npm run dev
    cd ..
}

function run_server() {
    echo "Starting the server (python3 main.py)..."
    cd backend || exit
    python3 main.py
    cd ..
}

function build_msgs() {
    build_mavlink_msgs
    build_ros_msgs
    source_ros
    wait
}

function run() {
    source_ros
    # run_mavlink_client &
    run_server &
    run_app &
    wait
}

function run_with_test_msgs() {
    source_ros
    run_server &
    run_app &
    publish_test_ros_msgs &
    wait
}

function run_all() {
    build_msgs
    run_docker_stack
    # run_mavlink_client &
    run_app &
    run_server &
    publish_test_ros_msgs &
    wait
}

function show_help() {
    echo "Usage: $0 [option]"
    echo "Options:"
    echo "  build_ros_msgs                 Build ROS 2 messages and gs_interfaces package"
    echo "  run_app                        Start the app (npm run dev)"
    echo "  run_server                     Start the server (python3 main.py)"
    echo "  publish_test_ros_msgs          Run custom messages (python3 tests/ros/run.py)"
    echo "  generate_mavlink               Generate MAVLink definitions using setup.sh"
    echo "  build_msgs                     Build MAVLink and ROS 2 messages"
    echo "  run_docker_stack [--cleanup]   Start InfluxDB + Grafana stack (with optional cleanup)"
    echo "  run                            Start the server and app"
    echo "  run_with_test_msgs             Start the server, app, and custom messages"
    echo "  run_all                        Build messages, start the server, app, and custom messages"
    echo "  help                           Show this help message"
}

if [ "$#" -lt 1 ]; then
    show_help
    exit 1
fi

case $1 in
    run_docker_stack)
        if [ "$2" = "--clean" ]; then
            run_docker_stack --clean
        else
            run_docker_stack
        fi
        ;;
    build_ros_msgs)
        build_ros_msgs
        ;;
    build_mavlink_msgs)
        build_mavlink_msgs
        ;;
    publish_test_ros_msgs)
        publish_test_ros_msgs
        ;;
    run_app)
        run_app
        ;;
    run_server)
        run_server
        ;;
    build_msgs)
        build_msgs
        ;;
    run)
        run
        ;;
    run_with_test_msgs)
        run_with_test_msgs
        ;;
    run_all)
        run_all
        ;;
    run_mavlink_receiver)
        run_mavlink_receiver
        ;;
    run_database)
        run_database
        ;;
    run_grafana)
        run_grafana
        ;;
    help)
        show_help
        ;;
    *)
        echo "Invalid option: $1"
        show_help
        exit 1
        ;;
esac