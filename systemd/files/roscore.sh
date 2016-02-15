#!/bin/bash

source /opt/ros/kinetic/setup.bash
export ROS_HOSTNAME=$(ifconfig eth0 | grep 'inet addr' | cut -d':' -f2 | awk '{print $1}')
export ROS_MASTER_URI="http://$ROS_HOSTNAME:5809"
roscore
