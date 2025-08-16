#!/bin/bash
set -e

echo "=== Loading DS3231 RTC kernel module ==="
sudo modprobe rtc-ds3231

# Ensure module loads on boot
if ! grep -q "rtc-ds3231" /etc/modules; then
    echo "rtc-ds3231" | sudo tee -a /etc/modules
fi

# Configure device tree overlay
CONFIG_FILE="/boot/firmware/config.txt"
if ! grep -q "dtoverlay=i2c-rtc,ds3231" "$CONFIG_FILE"; then
    echo "dtoverlay=i2c-rtc,ds3231" | sudo tee -a "$CONFIG_FILE"
fi

echo "=== Disabling fake-hwclock ==="
sudo apt remove -y fake-hwclock || true
sudo systemctl disable fake-hwclock || true

echo "=== Disabling NTP (systemd-timesyncd) ==="
sudo timedatectl set-ntp false
sudo systemctl stop systemd-timesyncd
sudo systemctl disable systemd-timesyncd

echo "=== Synchronizing system time with RTC ==="
# Read RTC time
sudo hwclock -r

# Set system time from RTC
sudo hwclock -s

echo "=== RTC setup complete ==="
echo "You can verify the RTC with:"
echo "  ls /dev/ | grep rtc"
echo "  sudo hwclock -r"

echo "=== Rebooting system to apply changes ==="
sudo reboot
