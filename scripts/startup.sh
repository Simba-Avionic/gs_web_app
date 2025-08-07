#!/bin/bash

set -o allexport
source "$HOME/gs_web_app/.env"
set +o allexport

until systemctl is-active --quiet simba-app; do
    sleep 1
done

while ! nc -z "$IP_ADDRESS" 2137; do
    sleep 1
done

chromium-browser --kiosk "http://$IP_ADDRESS:2137"


# grep -qxF 'dtoverlay=i2c-rtc,ds3231' /boot/firmware/config.txt || echo 'dtoverlay=i2c-rtc,ds3231' | sudo tee -a /boot/firmware/config.txt

# sudo hwclock -w

# sudo hwclock -r
