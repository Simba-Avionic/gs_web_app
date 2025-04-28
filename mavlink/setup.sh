#!/bin/bash

OUTPUT_FILE_NAME=simba
XML_FILE=simba_mavlink/simba.xml
OUTPUT_DIR=pymavlink/dialects/v10

# Initialize and update the submodule
git submodule update --init --recursive

# Run the mavgen command
python3 -m pymavlink.tools.mavgen --lang=Python --output=$OUTPUT_FILE_NAME $XML_FILE

# Check if the mavgen command was successful
if [ $? -ne 0 ]; then
    echo "mavgen command failed"
    echo "Make sure you initialized the submodules correctly"
    exit 1
fi

# Copy the generated files to the output directory
cp $OUTPUT_FILE_NAME.py $OUTPUT_DIR/$OUTPUT_FILE_NAME.py
cp simba_mavlink/$OUTPUT_FILE_NAME.xml $OUTPUT_DIR/$OUTPUT_FILE_NAME.xml

cd pymavlink

# Setup pymavlink libraries
python3 setup.py install --user

echo "Setup completed successfully"