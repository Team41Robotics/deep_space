#!/usr/bin/env python
import rospy
from networktables import NetworkTables
from sensor_msgs.msg import LaserScan
from laser_line_extraction.msg import LineSegment, LineSegmentList
from std_msgs.msg import Float64
from math import pi

sd = None # SmartDashboard instance
pub = rospy.Publisher('filter_scan', LaserScan, queue_size=10) # Filtered scan topic from -45 to 45 degrees
angle_pub = rospy.Publisher('debug_angle', Float64, queue_size = 1)
dist_pub = rospy.Publisher('debug_distance', Float64, queue_size = 1)

def filter_scan(msg, start_angle, end_angle):
	start_index = int((start_angle - msg.angle_min) / msg.angle_increment)
	end_index = int((end_angle - msg.angle_min) / msg.angle_increment)
	data = list(msg.ranges)
	for i in range(len(msg.ranges)):
		if i < start_index or i > end_index:
			data[i] = 0
	msg.ranges = data
	pub.publish(msg)

def line_callback(msg):
	if len(msg.line_segments) > 0:
		line = msg.line_segments[0] # Placeholder for best line
		for seg in msg.line_segments: # Look for line with angle closest to 0
			if abs(seg.angle) < abs(line.angle):
				line = seg
		rospy.loginfo('Best line: radius ' + str(line.radius) + ', angle ' + str(line.angle))
		angle_pub.publish(Float64(line.angle*180/pi))
		dist_pub.publish(Float64(line.radius))
		sd.putNumber('Angle of Line',line.angle*180/pi)
		sd.putNumber('Distance to Line',line.radius * 39.3701)
		est = sd.getNumber('Estimated Distance', 0)
		err = (line.radius * 39.3701) - est
		if err < 15:
			sd.putNumber('Actual minus Estimated', err)

def callback(msg):
	# A silly little exercise
	rospy.loginfo('min lidar index and distance')
	index = 0
	m = max(msg.ranges) # Minimum
	for i in range(len(msg.ranges)):
		if msg.ranges[i] < m and msg.ranges[i] > 0:
			m = msg.ranges[i]
			index = i
	rospy.loginfo('{}:\t {}'.format(index,m))
	sd.putNumber('Min index',index)
	# Publish filter scan
	filter_scan(msg, -pi/4, pi/4)
	rospy.loginfo('Published filter scan')

def getLidarData():
	global sd
	rospy.loginfo('Initializing network tables')
	NetworkTables.initialize(server='10.0.41.2')
	sd = NetworkTables.getTable("SmartDashboard")
	rospy.loginfo('NetworkTables initialized')
	rospy.Subscriber('scan',LaserScan,callback)
	rospy.loginfo('Subscribed to scan')
	rospy.Subscriber('line_segments', LineSegmentList, line_callback)
	rospy.loginfo('Subscribed to line extraction')

def main():
	rospy.init_node('lidar_processor')
	rospy.loginfo('starting lidar processor')
	getLidarData()
	rospy.spin()

if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.loginfo('interrupt caught')
		f.close()
