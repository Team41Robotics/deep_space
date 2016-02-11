#!/bin/bash

sleep 5
source /opt/ros/kinetic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash
hostname=$(ifconfig eth0 | grep 'inet addr' | cut -d':' -f2 | awk '{print $1}')
if [ -z "$hostname" ]; then
	export ROS_HOSTNAME=10.0.41.101
	export ROS_MASTER_URI="http://$ROS_HOSTNAME:5809"
else
	export ROS_HOSTNAME=$hostname
	export ROS_MASTER_URI="http://$ROS_HOSTNAME:5809"
fi

roslaunch deep_space all.launch
