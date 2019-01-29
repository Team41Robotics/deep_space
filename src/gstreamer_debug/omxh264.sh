#!/usr/bin/env bash

PORT=$1
TARGET_IP=10.0.41.109
FPS=20

echo Sending /dev/video$PORT to $TARGET_IP on port 580$PORT

gst-launch-1.0 -v v4l2src device=/dev/video$PORT ! "video/x-raw,width=640,height=480,framerate=$FPS/1" ! omxh264enc bitrate=1500 iframeinterval=2  ! rtph264pay ! udpsink host=$TARGET_IP port=580$PORT
