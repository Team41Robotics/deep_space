#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash
export ROS_HOSTNAME=
hostname=$(ifconfig eth0 | grep 'inet addr' | awk '{print $2}' | awk -F':' '{print $2}')
if [ -z "$hostname" ]; then
	export ROS_HOSTNAME=10.0.41.101
	export ROS_MASTER_URI="http://$ROS_HOSTNAME:11311"
else
	export ROS_HOSTNAME=$hostname
	export ROS_MASTER_URI="http://$ROS_HOSTNAME:11311"
fi

roslaunch deep_space all.launch
