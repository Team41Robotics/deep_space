#!/usr/bin/env python
import rospy
from networktables import NetworkTables
from sensor_msgs.msg import LaserScan

import datetime # For the print to sd to show something new every deploy

sd = None

def callback(msg):
	rospy.loginfo('lidar front')
	rospy.loginfo(msg.ranges[0])
	rospy.loginfo('min lidar index and distance')
	index = 0
	m = max(msg.ranges) # Minimum
	for i in range(len(msg.ranges)):
		if msg.ranges[i] < m and msg.ranges[i] > 0:
			m = msg.ranges[i]
			index = i
	rospy.loginfo('{}:\t {}'.format(index,m))
	sd.putNumber('testnum',index)

def getLidarData():
	global sd
	rospy.loginfo('Initializing network tables')
	NetworkTables.initialize(server='10.0.41.2')
	sd = NetworkTables.getTable("SmartDashboard")
	rospy.loginfo('NetworkTables initialized')
	rospy.Subscriber('scan',LaserScan,callback) # Callback not being called
	# No publisher to scan topic, use rostopic info /scan to see
	rospy.loginfo('Subscribed to scan')

def main():
	rospy.init_node('lidar_processor')
	rospy.loginfo('starting lidar processor')
	getLidarData()
	rospy.loginfo('Printing to sd')
	rospy.spin() # Does not get past this method call

if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.loginfo('interrupt caught')
