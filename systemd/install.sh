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

echo "Done."
