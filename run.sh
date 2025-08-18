#!/bin/bash

# You can adjust the APP_PATH if app is installed in a different location
APP_PATH="$HOME"
# echo "$APP_PATH"
BASHRC="$HOME/.bashrc"
LINE="source $APP_PATH/gs_web_app/build/install/setup.bash"

function source_venv() {
    source $APP_PATH/gs_web_app/venv/bin/activate
    if [ $? -ne 0 ]; then
        echo "Failed to source Python virtual environment."
        exit 1
    fi
}

function source_ros() {
    source $APP_PATH/gs_web_app/build/install/setup.bash
    if [ $? -ne 0 ]; then
        echo "Failed to source ROS 2 environment."
        exit 1
    fi
}

function run_oled_display() {
    echo "Starting OLED display script (python3 oled_display.py)..."
    source_venv
    python3 $APP_PATH/gs_web_app/oled_display.py
    if [ $? -ne 0 ]; then
        echo "Failed to start OLED display script."
        exit 1
    fi
}

function build_ros_msgs() {
    echo "Building ROS 2 messages..."

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
    python3 mavlink/mavlink_client.py
}

function run_database() {
    echo "Starting the database (docker-compose up -d)..."
    docker compose --env-file .env up -d influxdb
    if [ $? -ne 0 ]; then
        echo "Failed to start the database."
        exit 1
    fi
}

function run_docker_stack() {

    if [ "$1" = "--clean" ]; then
        echo "Cleaning up Docker containers first..."
        chmod +x ./scripts/cleanup_docker.sh
        # ./scripts/cleanup_docker.sh grafana
        ./scripts/cleanup_docker.sh influxdb
        if [ $? -ne 0 ]; then
            echo "Docker cleanup failed."
            exit 1
        fi
    fi
    
    # Check for offline Docker images in the docker directory
    if [ -d "./docker" ]; then
        echo "Checking for offline Docker images in docker directory..."
        IMAGE_COUNT=0
        for image_file in ./docker/*.tar ./docker/*.tar.gz ./docker/*.tgz; do
            if [ -f "$image_file" ]; then
                echo "Loading Docker image from $image_file..."
                docker load -i "$image_file"
                if [ $? -eq 0 ]; then
                    ((IMAGE_COUNT++))
                    echo "Successfully loaded image from $image_file"
                else
                    echo "Failed to load image from $image_file"
                fi
            fi
        done
        
        if [ $IMAGE_COUNT -gt 0 ]; then
            echo "Loaded $IMAGE_COUNT offline Docker images"
        else
            echo "No offline Docker images found in docker directory"
        fi
    fi

    echo "Starting InfluxDB..."
    docker compose --env-file .env up -d
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
    source_venv
    source_ros
    run_docker_stack &
    # run_mavlink_client &
    run_server &
    run_app &
    wait
}

function run_with_test_msgs() {
    run
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
    run_mavlink_receiver)
        run_mavlink_receiver
        ;;
    run_database)
        run_database
        ;;
    run_grafana)
        run_grafana
        ;;
    run_oled_display)
        run_oled_display
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