#!/bin/bash

# Check if the correct number of arguments are provided
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <output_file> <xml_file>"
    exit 1
fi

OUTPUT_FILE_NAME=$1
XML_FILE=$2
OUTPUT_DIR=pymavlink/dialects/v10

# Initialize and update the submodule
git submodule update --init --recursive

# Run the mavgen command
python3 -m pymavlink.tools.mavgen --lang=Python --output=$OUTPUT_FILE_NAME $XML_FILE

# Check if the mavgen command was successful
if [ $? -ne 0 ]; then
    echo "mavgen command failed"
    exit 1
fi

# Copy the generated files to the output directory
cp $OUTPUT_FILE_NAME.py $OUTPUT_DIR/$OUTPUT_FILE_NAME.py
cp message_definitions/v1.0/$OUTPUT_FILE_NAME.xml $OUTPUT_DIR/$OUTPUT_FILE_NAME.xml

cd pymavlink

# Setup pymavlink libraries
python3 setup.py install --user

echo "Setup completed successfully"