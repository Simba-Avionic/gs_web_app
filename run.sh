#!/bin/bash

function build_ros_messages() {
    echo "Building ROS 2 messages..."
    cd gs_interfaces || exit
    python3 generate_ros2_messages.py
    if [ $? -ne 0 ]; then
        echo "Failed to generate ROS 2 messages."
        exit 1
    fi

    cd ..

    colcon build --packages-select gs_interfaces
    if [ $? -ne 0 ]; then
        echo "Failed to build gs_interfaces package."
        exit 1
    fi
    echo "ROS 2 messages built successfully."
}

function generate_mavlink_definitions() {
    echo "Generating MAVLink definitions using setup.sh..."
    cd mavlink|| exit
    chmod +x setup.sh
    ./setup.sh simba message_definitions/v1.0/simba.xml
    if [ $? -ne 0 ]; then
        echo "Failed to generate MAVLink definitions."
        exit 1
    fi
    cd ..
    echo "MAVLink definitions generated successfully."
}

function run_custom_ros_messages() {
    echo "Running custom messages (python3 sim_nodes/run.py)..."
    python3 sim_nodes/run.py
}

function run_mavlink_receiver() {
    echo "Running MAVLink receiver... (python3 mavlink/receiver.py)"
    python3 mavlink/receiver.py

    if [ $? -ne 0 ]; then
        echo "Failed to run MAVLink receiver."
        exit 1
    fi
}

function run_database() {
    cd server/database || exit
    echo "Starting the database (docker-compose up -d)..."
    sudo docker compose --env-file ../../.env up -d
    if [ $? -ne 0 ]; then
        echo "Failed to start the database."
        exit 1
    fi
    cd ../..
}

function run_grafana() {
    cd grafana || exit
    echo "Generating Grafana dashboards..."
    python3 generate_dashboards.py
    
    echo "Starting Grafana (docker-compose up -d)..."
    sudo docker compose up -d
    
    cd ..
}

function run_app() {
    echo "Starting the app (npm run dev)..."
    cd app || exit
    npm run dev
    cd ..
}

function run_server() {
    echo "Starting the server (python3 main.py)..."
    cd server || exit
    python3 main.py
    cd ..
}

function build_msgs() {
    generate_mavlink_definitions
    build_ros_messages
    wait
}

function run() {
    run_mavlink_receiver &
    run_server &
    run_app &
    wait
}

function run_with_test_msgs() {
    run_server &
    run_app &
    run_custom_ros_messages &
    wait
}

function run_all() {
    build_msgs
    run_database
    run_grafana
    run_mavlink_receiver &
    run_app &
    run_server &
    run_custom_ros_messages &
    wait
}

function show_help() {
    echo "Usage: $0 [option]"
    echo "Options:"
    echo "  build_ros_messages       Build ROS 2 messages and gs_interfaces package"
    echo "  run_app                  Start the app (npm run dev)"
    echo "  run_server               Start the server (python3 main.py)"
    echo "  run_custom_ros_messages  Run custom messages (python3 sim_nodes/run.py)"
    echo "  generate_mavlink         Generate MAVLink definitions using setup.sh"
    echo "  build_msgs               Build MAVLink and ROS 2 messages"
    echo "  run                      Start the server and app"
    echo "  run_with_test_msgs       Start the server, app, and custom messages"
    echo "  run_all                  Build messages, start the server, app, and custom messages"
    echo "  help                     Show this help message"
}

# Main script logic
if [ "$#" -lt 1 ]; then
    show_help
    exit 1
fi

case $1 in
    build_ros_messages)
        build_ros_messages
        ;;
    generate_mavlink)
        generate_mavlink_definitions
        ;;
    run_custom_ros_messages)
        run_custom_ros_messages
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