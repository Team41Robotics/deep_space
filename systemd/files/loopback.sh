#!/bin/bash

sudo modprobe v4l2loopback video_nr=5,6,7 exclusive_caps=1 max_buffers=2

until ping -c1 -W 1 10.0.41.5 &>/dev/null; do sleep 1; done
echo Found it
