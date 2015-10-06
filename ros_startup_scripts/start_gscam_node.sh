#!/bin/sh

#export GSCAM_CONFIG="v4l2src device=/dev/video0 ! video/x-raw-rgb,framerate=30/1 ! ffmpegcolor space"
#/opt/ros/indigo/lib/gscam/gscam &
roslaunch gscam.launch & 
