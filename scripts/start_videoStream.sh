#!/bin/bash
exec raspivid -co 50 -br 65 -awb auto --nopreview -t 0 -vf --profile baseline  --flush -ih -h 240 -w 320 -fps 10 -hf -b 90000 -o - \
 | gst-launch-1.0 fdsrc do-timestamp=true ! h264parse ! rtph264pay config-interval=5 pt=96 ! udpsink host=160.85.37.148 port=8004 sync=false
#host=160.85.37.142 port=30971 sync=false 
