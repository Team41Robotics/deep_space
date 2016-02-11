# Peter's Second Gstreamer Tutorial

## MJPEG Stream from fisheye cam

### Server side

gst-launch-1.0 -v v4l2src device=/dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0 ! jpegdec ! videoconvert ! videoscale ! video/x-raw,width=320,height=240,framerate=30/1 ! jpegenc ! rtpjpegpay ! udpsink host=10.0.41.156 port=5800

### Client

gst-launch-1.0 -e -v udpsrc port=5800 ! application/x-rtp, encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! autovideosink

## H.264 Stream from fisheye cam

### Server side

gst-launch-1.0 -v v4l2src device=/dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0 ! jpegdec ! videoconvert ! videoscale ! video/x-raw,width=320,height=240,framerate=30/1 ! x264enc tune=zerolatency bitrate=1200 speed-preset=ultrafast key-int-max=10 ip-factor=.5 ! rtph264pay ! udpsink host=10.0.41.101 port=5800 buffer-size=5

gst-launch-1.0 -v udpsrc port=5800 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! autovideosink
