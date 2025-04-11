#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: $0 <name>"
    echo "Provide the name of the Docker containers/images to stop and delete."
    exit 1
fi

NAME=$1

echo "Stopping and removing Docker containers with name containing '$NAME'..."

sudo docker ps -a --filter "name=$NAME" --format "{{.ID}}" | while read -r container_id; do
    echo "Stopping container $container_id..."
    sudo docker stop "$container_id"
    echo "Removing container $container_id..."
    sudo docker rm "$container_id"
done

echo "Removing Docker images with name containing '$NAME'..."

sudo docker images --filter "reference=$NAME*" --format "{{.ID}}" | while read -r image_id; do
    echo "Removing image $image_id..."
    sudo docker rmi "$image_id"
done

echo "Cleanup completed."