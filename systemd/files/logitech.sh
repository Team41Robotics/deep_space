#!/bin/bash

sleep 5
gst-launch-1.0 -v v4l2src \
device=/dev/video5 \
! videoscale ! videorate \
! video/x-raw,width=320,height=240,framerate=15/1 \
! x264enc bitrate=1024 tune=zerolatency \
! rtph264pay ! udpsink host=10.0.41.5 port=5801 \
