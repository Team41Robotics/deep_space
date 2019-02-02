#!/bin/bash

echo "Let's try waiting"
sleep 30

# Streaming function
stream_cam () {
	gst-launch-1.0 -v v4l2src device=/dev/v4l/by-id/$1 \
	! "video/x-raw,width=640,height=480,framerate=15/1" \
	! x264enc tune=zerolatency bitrate=1500 speed-preset=ultrafast \
	key-int-max=2 ip-factor=0.5 \
	! rtph264pay ! udpsink host=10.0.41.109 port=$2
}

# Main driving camera
stream_cam usb-046d_081d_731EE4C0-video-index0 5800 &

# Supp driving camera
stream_cam usb-046d_HD_Webcam_C525_C9EE2A60-video-index0 5801
