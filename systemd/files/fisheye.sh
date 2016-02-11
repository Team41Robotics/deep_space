#!/bin/bash

gst-launch-1.0 -v v4l2src \
device=/dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0 \
! jpegdec \
! videoscale ! videorate \
! video/x-raw,width=320,height=240,framerate=12/1 \
! x264enc bitrate=256 tune=zerolatency \
! rtph264pay ! udpsink host=10.0.41.156 port=5800 \
