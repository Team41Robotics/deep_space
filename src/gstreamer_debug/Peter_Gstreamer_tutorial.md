# Gstreamer webcam stream tutorial

## MPJEG

### Server side:

gst-launch-1.0 -v v4l2src device=/dev/video0 ! "image/jpeg,width=1280, height=720,framerate=30/1" ! rtpjpegpay ! udpsink host=127.0.0.1 port=5800

### Client side:

gst-launch-1.0 -e -v udpsrc port=5800 ! application/x-rtp, encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! autovideosink

## H.264

### Server side:

gst-launch-1.0 -v v4l2src device=/dev/video0 ! "video/x-raw,width=640,height=480,framerate=30/1" ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ! udpsink host=$TARGET_IP port=5800

### Client side:

gst-launch-1.0 -v udpsrc port=5800 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! autovideosink

## UDP Multicast

### Server side:

gst-launch-1.0 -v v4l2src device=/dev/video0 ! "video/x-raw,width=640,height=480,framerate=30/1" ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ! udpsink host=224.0.0.3 port=5800 auto-multicast=true

### Client side:

gst-launch-1.0 -v udpsrc port=5800 auto-multicase=true address=224.0.0.3 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! autovideosink

## Parameters for H.264 encoder

[x264enc](https://gstreamer.freedesktop.org/data/doc/gstreamer/head/gst-plugins-ugly-plugins/html/gst-plugins-ugly-plugins-x264enc.html#GstX264Enc--ip-factor)
