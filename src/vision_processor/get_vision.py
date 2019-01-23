#!/usr/bin/env python
import rospy
from networktables import NetworkTables
#from deep_space.msg import PoseXY
import vision_estimation

pub = rospy.Publisher('vision_pose', PoseXY, queue_size = 1)

def get_vision_data():
	pose = vision_estimation.get_camera_data()
	poseXY = PoseXY()
	poseXY.x = pose[0]
	poseXY.y = pose[1]
	pub.publish(poseXY)

def main():
	rospy.init_node('vision_processor')
	rospy.loginfo('starting vision processor')
	while True:
		get_vision_data()
	rospy.spin()
	
if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.loginfo('interrupt caught')
		vision_estimation.camera_interrupt()
