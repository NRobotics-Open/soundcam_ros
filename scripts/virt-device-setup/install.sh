#!/bin/bash

set -e  # Exit immediately if a command exits with a non-zero status

# Define the service file path
SERVICE_FILE="/etc/systemd/system/v4l2loopback.service"

# Define the modprobe options
MODULE_OPTIONS="devices=1 video_nr=1 card_label='SoundCamVirtual'"

# Check if the script is run as root
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root" >&2
   exit 1
fi

# Ensure v4l2loopback module is installed
echo "Installing v4l2loopback-dkms..."
apt-get update && apt-get install -y v4l2loopback-dkms

# Create the systemd service file
echo "Creating the systemd service file at $SERVICE_FILE..."
cat <<EOF > $SERVICE_FILE
[Unit]
Description=Load v4l2loopback module with options
After=network.target

[Service]
Type=oneshot
ExecStart=/sbin/modprobe v4l2loopback $MODULE_OPTIONS
RemainAfterExit=true

[Install]
WantedBy=multi-user.target
EOF

# Reload systemd and enable the service
echo "Reloading systemd and enabling the v4l2loopback service..."
systemctl daemon-reload
systemctl enable v4l2loopback.service

# Start the service
echo "Starting the v4l2loopback service..."
systemctl start v4l2loopback.service

# Verify the service status
echo "Verifying the service status..."
systemctl status v4l2loopback.service

# Check if the module is loaded
echo "Checking if the v4l2loopback module is loaded..."
if lsmod | grep -q v4l2loopback; then
    echo "v4l2loopback module is successfully loaded."
    v4l2-ctl --list-devices
else
    echo "Failed to load the v4l2loopback module." >&2
    exit 1
fi

# Done
echo "v4l2loopback service setup completed successfully."