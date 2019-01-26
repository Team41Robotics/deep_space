#!/usr/bin/env python
import rospy
from networktables import NetworkTables
from deep_space.msg import PoseXY
import vision_estimation

pub = rospy.Publisher('vision_pose', PoseXY, queue_size = 1)
debug_window = False

def get_vision_data(debug_window):
	pose = vision_estimation.get_camera_data(debug_window)
	if pose is not None:
		poseXY = PoseXY()
		poseXY.x = pose[0]
		poseXY.y = pose[1]
		pub.publish(poseXY)

def main():
	global debug_window
	rospy.init_node('vision_processor')
	rospy.loginfo('starting vision processor')
        debug_window = rospy.get_param('~debug_window')
	print("THIS IS THE DEBUG WINDOW PARAM")
	print(debug_window)
	while not vision_estimation.get_camera_interrupt():
		get_vision_data(debug_window)
		
	
if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.loginfo('interrupt caught')
