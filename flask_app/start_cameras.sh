#!/bin/bash
# Camera 0
mjpg_streamer -i "input_uvc.so -d /dev/video0 -r 288x160 -f 5" \
              -o "output_http.so -p 8080 -w /usr/share/mjpg-streamer/www" &

# Camera 4
mjpg_streamer -i "input_uvc.so -d /dev/video4 -r 320x240 -f 5" \
              -o "output_http.so -p 8081 -w /usr/share/mjpg-streamer/www" &
