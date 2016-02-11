#!/usr/bin/env python
import rospy
from networktables import NetworkTables
from sensor_msgs.msg import LaserScan
from laser_line_extraction.msg import LineSegment, LineSegmentList
from std_msgs.msg import Float64
from math import pi
import tf

sd = None # SmartDashboard instance
pub = rospy.Publisher('filter_scan', LaserScan, queue_size=10) # Filtered scan topic from -45 to 45 degrees
angle_pub = rospy.Publisher('lidar_angle', Float64, queue_size = 1)
dist_pub = rospy.Publisher('debug_distance', Float64, queue_size = 1)
lidar_model = 'ydlidar'
tf_broadcaster = tf.TransformBroadcaster()
line_angle = 0 

def meters(inches):
	return inches*.0254

def broadcast_tf(event):
	tf_broadcaster.sendTransform((meters(-10),meters(-5),0),tf.transformations.quaternion_from_euler(0,0,pi/2),rospy.Time.now(),'base_lidar','base_link')
	tf_broadcaster.sendTransform((meters(-10),meters(-5),0),tf.transformations.quaternion_from_euler(0,0,line_angle),rospy.Time.now(),'base_camera','base_link')

def filter_scan(msg, start_angle, end_angle):
	start_index = int((start_angle - msg.angle_min) / msg.angle_increment)
	end_index = int((end_angle - msg.angle_min) / msg.angle_increment)
	data = list(msg.ranges)
	for i in range(len(msg.ranges)):
		if i < start_index or i > end_index or msg.ranges[i] > 2.0:
			data[i] = 0
	msg.ranges = data
	msg.scan_time = 0.0833 #1/12 #12Hz frequency means 1/12 seconds per scan
	msg.time_increment = 0.0001157 #1/12/360/0.5 #scan_time divided by scans (360 degrees / scan for every half deg
	pub.publish(msg)

def line_callback(msg):
	global line_angle
	angle_offset = pi/2 
	if lidar_model == 'rplidar':
		angle_offset = 0
	if len(msg.line_segments) > 0:
		line = msg.line_segments[0] # Placeholder for best line
		for seg in msg.line_segments: # Look for line with angle closest to 0
			if abs(seg.angle+angle_offset) < abs(line.angle+angle_offset):
				line = seg
		dist_pub.publish(Float64(line.radius))
		line_angle = float(line.angle+angle_offset)
		angle_pub.publish(Float64(line_angle))
		sd.putNumber('Angle of Line',(line_angle)*180/pi)

def callback(msg):
	# Publish filter scan
	if lidar_model == 'rplidar':
		filter_scan(msg, -pi/4, pi/4)
	elif lidar_model == 'ydlidar':	
		filter_scan(msg, -3*pi/4, -pi/4)
	else:
		rospy.loginfo('Lidar model ' + lidar_model + ' is not supported in the current version of deep_space')

def get_lidar_data():
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
	global lidar_model
	rospy.init_node('lidar_processor')
	rospy.loginfo('starting lidar processor')
	lidar_model = rospy.get_param('~lidar_model')
	rospy.Timer(rospy.Duration(.1), broadcast_tf)
	get_lidar_data()
	rospy.spin()

if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.loginfo('interrupt caught')
		#f.close()
