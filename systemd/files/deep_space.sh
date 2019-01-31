#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash
export ROS_HOSTNAME=$(ifconfig eth0 | grep 'inet addr' | awk '{print $2}' | awk -F':' '{print $2}')
export ROS_MASTER_URI="http://$ROS_HOSTNAME:11311"

roslaunch deep_space all.launch
