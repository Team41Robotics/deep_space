#!/bin/bash

source /opt/ros/kinetic/setup.bash
export ROS_HOSTNAME=$(ifconfig eth0 | grep 'inet addr' | awk '{print $2}' | awk -F':' '{print $2}')

roscore
