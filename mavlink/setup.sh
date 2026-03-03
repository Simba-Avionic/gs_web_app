#!/bin/bash

OUTPUT_FILE=src/simba
XML_FILE=simba_mavlink/simba.xml
OUTPUT_DIR=pymavlink/dialects/v10
MSG_DEF_DIR=message_definitions/v1.0

# Path to the actual XML to be compiled
OUTPUT=message_definitions/v1.0/simba.xml

# git submodule update --init --recursive

mkdir -p $MSG_DEF_DIR

# --- Copy all XML files from the source directory ---
SOURCE_XML_DIR=$(dirname "$XML_FILE")
cp "$SOURCE_XML_DIR"/*.xml "$MSG_DEF_DIR/"

python3 -m pymavlink.tools.mavgen --lang=Python --output=$OUTPUT_FILE $OUTPUT

if [ $? -ne 0 ]; then
    echo "mavgen command failed"
    echo "Make sure you initialized the submodules correctly"
    exit 1
fi

# --- Copy all XML files to the dialect output directory ---
mkdir -p "$OUTPUT_DIR"
cp "$MSG_DEF_DIR"/*.xml "$OUTPUT_DIR/"

cd pymavlink

python3 setup.py install --user

echo "Setup completed successfully"