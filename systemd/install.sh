#!/bin/bash

echo "Installing necessary services..."

cp ./files/roscore.sh /usr/local/bin/
cp ./files/roscore.service /etc/systemd/system/
echo "  Installed roscore.service"

cp ./files/deep_space.sh /usr/local/bin/
cp ./files/deep_space.service /etc/systemd/system/
echo "  Installed deep_space.service"

cp ./files/gstreamer.sh /usr/local/bin/
cp ./files/gstreamer.service /etc/systemd/system/
echo "  Installed gstreamer.service"

echo "Refreshing systemctl daemon..."
systemctl daemon-reload

echo "Starting services..."
systemctl restart roscore
echo "  Started roscore.service"
systemctl restart deep_space
echo "  Started deep_space.service"
systemctl restart gstreamer
echo "  Started gstreamer.service"

echo "Enabling services on reboot..."
systemctl enable roscore
echo "  Enabled roscore.service"
systemctl enable deep_space
echo "  Enabled deep_space.service"
systemctl enable gstreamer
echo "  Enabled gstreamer.service"

echo "Done."
