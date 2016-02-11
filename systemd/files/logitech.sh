#!/bin/bash

echo "Let's try waiting"
sleep 1

gst-launch-1.0 -v v4l2src device=/dev/v4l/by-id/usb-046d_081d_731EE4C0-video-index0 \
! videoscale ! videorate ! video/x-raw,width=320,height=240,framerate=12/1 \
! x264enc tune=zerolatency bitrate=256 \
! rtph264pay !  udpsink host=10.0.41.156 port=5801
