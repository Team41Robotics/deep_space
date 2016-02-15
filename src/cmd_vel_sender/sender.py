#!/usr/bin/env python
import rospy
from networktables import NetworkTables
from geometry_msgs.msg import Twist 
from math import pi

sd = None # SmartDashboard instance

def callback(msg):
	lin_vel = msg.linear.x 
	ang_vel = msg.angular.z 
	sd.putNumber('target_linear_velocity',lin_vel)
	sd.putNumber('target_angular_velocity',ang_vel)

def main():
	global sd
	rospy.init_node('cmd_vel_sender')
	rospy.loginfo('starting cmd_vel sender')
	NetworkTables.initialize(server='10.0.41.2')
	rospy.loginfo('Initializing network tables')
	sd = NetworkTables.getTable("SmartDashboard")
	rospy.loginfo('NetworkTables initialized')
	rospy.Subscriber('cmd_vel',Twist,callback)
	rospy.spin()

if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.loginfo('interrupt caught')
