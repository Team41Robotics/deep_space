#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from math import pi

def callback(msg):
	min_index = msg.ranges.index(max(msg.ranges))
	for i in range(len(msg.ranges)):
		if msg.ranges[i] > 0 and msg.ranges[i] < msg.ranges[min_index]:
			min_index = i
	min_angle = min_index * msg.angle_increment + msg.angle_min
	print('{0:.2f}'.format(min_angle * 180.0 / pi))

def main():
	rospy.init_node('lidar_processor')
	rospy.Subscriber('scan',LaserScan,callback)
	rospy.spin()

if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.loginfo('interrupt caught')
