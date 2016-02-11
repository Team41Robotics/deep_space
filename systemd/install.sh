#!/bin/bash

echo "Installing necessary services..."

cp ./files/roscore.sh /usr/local/bin/
cp ./files/roscore.service /etc/systemd/system/
echo "  Installed roscore.service"

cp ./files/deep_space.sh /usr/local/bin/
cp ./files/deep_space.service /etc/systemd/system/
echo "  Installed deep_space.service"

cp ./files/fisheye.sh /usr/local/bin/
cp ./files/fisheye.service /etc/systemd/system/
echo "  Installed fisheye.service"
cp ./files/logitech.sh /usr/local/bin/
cp ./files/logitech.service /etc/systemd/system/
echo "  Installed logitech.service"

echo "Refreshing systemctl daemon..."
systemctl daemon-reload

echo "Starting services..."
systemctl restart roscore
echo "  Started roscore.service"
systemctl restart deep_space
echo "  Started deep_space.service"
systemctl restart fisheye
echo "  Started fisheye.service"
systemctl restart logitech
echo "  Started logitech.service"

echo "Enabling services on reboot..."
systemctl enable roscore
echo "  Enabled roscore.service"
systemctl enable deep_space
echo "  Enabled deep_space.service"
systemctl enable fisheye
echo "  Enabled fisheye.service"
systemctl enable logitech
echo "  Enabled logitech.service"

echo "Done."
