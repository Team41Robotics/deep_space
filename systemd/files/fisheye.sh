#!/bin/bash

gst-launch-1.0 -v v4l2src \
device="$2" \
! jpegdec \
! videoscale ! videorate \
! video/x-raw,width=640,height=480,framerate=15/1 \
! x264enc bitrate=1024 tune=zerolatency \
! rtph264pay ! udpsink host=10.0.41.156 port=$1 \
