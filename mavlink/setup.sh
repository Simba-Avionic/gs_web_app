#!/bin/bash

OUTPUT_FILE_NAME=simba
XML_FILE=simba_mavlink/simba.xml
OUTPUT_DIR=pymavlink/dialects/v10
MSG_DEF_DIR=message_definitions/v1.0

OUTPUT=message_definitions/v1.0/simba.xml

git submodule update --init --recursive

mkdir -p $MSG_DEF_DIR
cp $XML_FILE $MSG_DEF_DIR/

python3 -m pymavlink.tools.mavgen --lang=Python --output=$OUTPUT_FILE_NAME $OUTPUT

if [ $? -ne 0 ]; then
    echo "mavgen command failed"
    echo "Make sure you initialized the submodules correctly"
    exit 1
fi

# cp $MSG_DEF_DIR/$OUTPUT_FILE_NAME.py $OUTPUT_DIR/$OUTPUT_FILE_NAME.py
# cp simba_mavlink/$OUTPUT_FILE_NAME.xml $OUTPUT_DIR/$OUTPUT_FILE_NAME.xml

# Copy the generated files to the output directory
cp -r $MSG_DEF_DIR/* $OUTPUT_DIR/

cd pymavlink

# Setup pymavlink libraries
python3 setup.py install --user

echo "Setup completed successfully"