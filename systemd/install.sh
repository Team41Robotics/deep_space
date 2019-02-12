#!/bin/bash

echo "Installing necessary services..."

cp ./files/roscore.sh /usr/local/bin/
cp ./files/roscore.service /etc/systemd/system/
echo "  Installed roscore.service"

cp ./files/deep_space.sh /usr/local/bin/
cp ./files/deep_space.service /etc/systemd/system/
echo "  Installed deep_space.service"

cp ./files/fisheye.sh /usr/local/bin/
cp ./files/fisheye1.service /etc/systemd/system/
echo "  Installed fisheye1.service"
cp ./files/fisheye2.service /etc/systemd/system/
echo "  Installed fisheye2.service"

echo "Refreshing systemctl daemon..."
systemctl daemon-reload

echo "Starting services..."
systemctl restart roscore
echo "  Started roscore.service"
systemctl restart deep_space
echo "  Started deep_space.service"
systemctl restart fisheye1
echo "  Started fisheye1.service"
systemctl restart fisheye2
echo "  Started fisheye2.service"

echo "Enabling services on reboot..."
systemctl enable roscore
echo "  Enabled roscore.service"
systemctl enable deep_space
echo "  Enabled deep_space.service"
systemctl enable fisheye1
echo "  Enabled fisheye1.service"
systemctl enable fisheye2
echo "  Enabled fisheye2.service"

echo "Done."
