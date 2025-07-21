#!/bin/bash

set -o allexport
source "../$(dirname "$0")/.env"
set +o allexport

chmod +x ./gs_web_app/run.sh
.gs_web_app/run.sh run

firefox --kiosk "http://$IP_ADDRESS:2137"