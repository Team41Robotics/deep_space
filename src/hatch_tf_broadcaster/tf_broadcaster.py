#!/usr/bin/env python
import rospy 
import tf_conversions
import tf2_ros 
import numpy as np
from deep_space.msg import PoseXY 
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped

pos_x, pos_y, theta = 0, 0, 0
tfBuffer,listener = None, None

def hatch_bl_tf():
	br = tf2_ros.TransformBroadcaster()
	try:
		bl2cam = tfBuffer.lookup_transform('base_link', 'base_camera', rospy.Time())
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		#TODO: add correct exception here
		return

	bl2cam_trans_mat = (bl2cam.transform.translation.x,bl2cam.transform.translation.y,bl2cam.transform.translation.z)
	bl2cam_rot_mat = [bl2cam.transform.rotation.x,bl2cam.transform.rotation.y,bl2cam.transform.rotation.z,bl2cam.transform.rotation.w]
	bl2cam_mat = tf_conversions.transformations.concatenate_matrices(tf_conversions.transformations.translation_matrix(bl2cam_trans_mat),tf_conversions.transformations.quaternion_matrix(bl2cam_rot_mat))

	h2cam_mat = tf_conversions.transformations.concatenate_matrices(tf_conversions.transformations.translation_matrix((pos_x,pos_y,0)),tf_conversions.transformations.euler_matrix(0,0,0))

	#hatch to base_link
	h2bl_mat = np.dot(h2cam_mat,np.linalg.inv(bl2cam_mat))
	h2bl_tran = tf_conversions.transformations.translation_from_matrix(h2bl_mat)
	h2bl_rot = tf_conversions.transformations.quaternion_from_matrix(h2bl_mat)

	h2bl = TransformStamped()
	h2bl.header.stamp = rospy.Time.now()
	h2bl.header.frame_id = 'hatch_frame'
	h2bl.child_frame_id = 'base_link'
	h2bl.transform.translation.x = h2bl_tran[0]
	h2bl.transform.translation.y = h2bl_tran[1]
	h2bl.transform.translation.z = h2bl_tran[2]
	h2bl.transform.rotation.x = h2bl_rot[0]
	h2bl.transform.rotation.y = h2bl_rot[1]
	h2bl.transform.rotation.z = h2bl_rot[2]
	h2bl.transform.rotation.w = h2bl_rot[3]
	br.sendTransform(h2bl)

def lidar_callback(msg): 
	global theta 
	theta = msg.data
	hatch_bl_tf()

def vision_callback(msg):
	global pos_x, pos_y
	pos_x = msg.x
	pos_y = msg.y 
	hatch_bl_tf()

def main():
	global tfBuffer,listener
	rospy.init_node('hatch_tf_broadcaster')
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	rospy.Subscriber('lidar_angle', Float64, lidar_callback)
	rospy.Subscriber('vision_pose', PoseXY, vision_callback)
	rospy.loginfo('started hatch transform publisher')
	rospy.spin()

if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.loginfo('interrupt caught')