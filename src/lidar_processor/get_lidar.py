#!/usr/bin/env python
import rospy
from networktables import NetworkTables
from sensor_msgs.msg import LaserScan

sd = None

def callback(msg):
	print('lidar front')
	print(msg.ranges[0])
	print('min lidar index and distance')
	index = 0
	m = max(msg.ranges) # Minimum
	for i in range(len(msg.ranges)):
		if msg.ranges[i] < m and msg.ranges[i] > 0:
			m = msg.ranges[i]
			index = i
	print('{}:\t {}'.format(index,m))
	sd.putNumber('testnum',index)

def getLidarData():
	global sd
	NetworkTables.initialize(server='10.0.41.62')
	sd = NetworkTables.getTable("SmartDashboard")
	rospy.Subscriber('scan',LaserScan,callback)

def main():
	rospy.init_node('lidar_processor')
	getLidarData()
	rospy.spin()

if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.loginfo('interrupt caught')
