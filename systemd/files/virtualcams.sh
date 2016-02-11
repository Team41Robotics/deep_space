#!/bin/bash

v4l2-ctl -d /dev/v4l/by-id/usb-046d_081d_731EE4C0-video-index0 --set-ctrl=exposure_auto=1
v4l2-ctl -d /dev/v4l/by-id/usb-046d_081d_731EE4C0-video-index0 --set-ctrl=exposure_absolute=150

gst-launch-1.0 v4l2src device=/dev/v4l/by-id/usb-046d_081d_731EE4C0-video-index0 \
! video/x-raw,width=640,height=480,framerate=30/1 \
! tee name=tp tp. \
! queue ! videobalance brightness=0.1 ! v4l2sink device=/dev/video5 tp. \
! queue ! v4l2sink device=/dev/video6
