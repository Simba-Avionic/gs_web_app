#!/bin/bash

OUTPUT_FILE=src/simba
XML_FILE=simba_mavlink/simba.xml
OUTPUT_DIR=pymavlink/dialects/v10
MSG_DEF_DIR=message_definitions/v1.0

OUTPUT=message_definitions/v1.0/simba.xml

# git submodule update --init --recursive

mkdir -p $MSG_DEF_DIR
cp $XML_FILE $MSG_DEF_DIR/

python3 -m pymavlink.tools.mavgen --lang=Python --output=$OUTPUT_FILE $OUTPUT

if [ $? -ne 0 ]; then
    echo "mavgen command failed"
    echo "Make sure you initialized the submodules correctly"
    exit 1
fi

# Copy the generated files to the output directory
cp -r $MSG_DEF_DIR/* $OUTPUT_DIR/

cd pymavlink

# Setup pymavlink libraries
python3 setup.py install --user

echo "Setup completed successfully"