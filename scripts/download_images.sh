#!/bin/bash

GRAFANA_IMAGE="grafana/grafana:11.2.3"
INFLUXDB_IMAGE="influxdb:2.6.1"

OUTPUT_DIR="./docker"

mkdir -p "$OUTPUT_DIR"

# Pull Docker images
echo "Pulling Grafana image: $GRAFANA_IMAGE"
docker pull "$GRAFANA_IMAGE"

echo "Pulling InfluxDB image: $INFLUXDB_IMAGE"
docker pull "$INFLUXDB_IMAGE"

# Save Docker images to files in the docker directory
echo "Saving Grafana image to $OUTPUT_DIR/grafana_image.tar"
docker save -o "$OUTPUT_DIR/grafana_image.tar" "$GRAFANA_IMAGE"

echo "Saving InfluxDB image to $OUTPUT_DIR/influxdb_image.tar"
docker save -o "$OUTPUT_DIR/influxdb_image.tar" "$INFLUXDB_IMAGE"

sudo chmod +r "$OUTPUT_DIR/grafana_image.tar"
sudo chmod +r "$OUTPUT_DIR/influxdb_image.tar"

echo "✅ Docker images saved to $OUTPUT_DIR."