#!/bin/sh

export ROS_PACKAGE_PATH=/opt/ros/indigo
export ROS_ROOT=/opt/ros/indigo
export PATH=$PATH:/opt/ros/indigo/bin
export LD_LIBRARY_PATH=/opt/ros/indigo/lib
export PYTHONPATH=/opt/ros/indigo/lib/python2.7/site-packages
export ROS_HOSTNAME=192.168.178.40
export ROS_MASTER_URI=http://192.168.178.40:11311
export CMAKE_PREFIX_PATH=/opt/ros/indigo
touch /opt/ros/indigo/.catkin
roscore &

