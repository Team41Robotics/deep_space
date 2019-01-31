#!/usr/bin/env bash

ID=$1
PORT=$2
TARGET_IP=10.0.41.109
FPS=15

echo Sending /dev/v4l/by-id/$ID to $TARGET_IP on port 580$PORT

gst-launch-1.0 -v v4l2src device=/dev/v4l/by-id/$ID ! "video/x-raw,width=640,height=480,framerate=$FPS/1" ! x264enc tune=zerolatency bitrate=1500 speed-preset=ultrafast key-int-max=2 ip-factor=0.5 ! rtph264pay ! udpsink host=$TARGET_IP port=580$PORT
