#!/usr/bin/env python
import rospy 
import tf_conversions
import tf2_ros 
import numpy as np
from deep_space.msg import PoseXY 
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from math import pi

pos_x, pos_y, theta = 0, 0, 0
tape_visible = False
tfBuffer,listener = None, None
h2odom = TransformStamped()
h2odom.header.frame_id = 'hatch_frame'
h2odom.child_frame_id = 'odom'
h2odom.transform.rotation.w = 1.0 
 
def hatch_bl_tf(event):
	global h2odom,tape_visible

	br = tf2_ros.TransformBroadcaster()
	if tape_visible is False:
		h2odom.header.stamp = rospy.Time.now()
        	br.sendTransform(h2odom)
		return

	try:
		bl2cam = tfBuffer.lookup_transform('base_link', 'base_camera', rospy.Time())
		odom2bl = tfBuffer.lookup_transform('odom', 'base_link', rospy.Time())
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		#TODO: add correct exception here
		return

	bl2cam_trans_mat = (bl2cam.transform.translation.x,bl2cam.transform.translation.y,bl2cam.transform.translation.z)
	bl2cam_rot_mat = [bl2cam.transform.rotation.x,bl2cam.transform.rotation.y,bl2cam.transform.rotation.z,bl2cam.transform.rotation.w]
	bl2cam_mat = tf_conversions.transformations.concatenate_matrices(tf_conversions.transformations.translation_matrix(bl2cam_trans_mat),tf_conversions.transformations.quaternion_matrix(bl2cam_rot_mat))

	h2cam_mat = tf_conversions.transformations.concatenate_matrices(tf_conversions.transformations.translation_matrix((pos_x,pos_y,0)),tf_conversions.transformations.euler_matrix(0,0,-pi))

	#hatch to base_link
	h2bl_mat = np.dot(h2cam_mat,np.linalg.inv(bl2cam_mat))

	odom2bl_trans_mat = (odom2bl.transform.translation.x,odom2bl.transform.translation.y,odom2bl.transform.translation.z)
	odom2bl_rot_mat = [odom2bl.transform.rotation.x,odom2bl.transform.rotation.y,odom2bl.transform.rotation.z,odom2bl.transform.rotation.w]
	odom2bl_mat = tf_conversions.transformations.concatenate_matrices(tf_conversions.transformations.translation_matrix(odom2bl_trans_mat),tf_conversions.transformations.quaternion_matrix(odom2bl_rot_mat))

	h2odom_mat = np.dot(h2bl_mat,np.linalg.inv(odom2bl_mat))

	h2odom_tran = tf_conversions.transformations.translation_from_matrix(h2odom_mat)
	h2odom_rot = tf_conversions.transformations.quaternion_from_matrix(h2odom_mat)

	h2odom = TransformStamped()
	h2odom.header.stamp = rospy.Time.now()
	h2odom.header.frame_id = 'hatch_frame'
	h2odom.child_frame_id = 'odom'
	h2odom.transform.translation.x = h2odom_tran[0]
	h2odom.transform.translation.y = h2odom_tran[1]
	h2odom.transform.translation.z = h2odom_tran[2]
	h2odom.transform.rotation.x = h2odom_rot[0]
	h2odom.transform.rotation.y = h2odom_rot[1]
	h2odom.transform.rotation.z = h2odom_rot[2]
	h2odom.transform.rotation.w = h2odom_rot[3]
        br.sendTransform(h2odom)
	tape_visible = False

def lidar_callback(msg): 
	global theta 
	theta = msg.data
	#hatch_bl_tf()

def vision_callback(msg):
	global pos_x, pos_y,tape_visible
	pos_x = msg.x
	pos_y = msg.y 
	#hatch_bl_tf()
	#print('x: '+str(pos_x)+'\ty: '+str(pos_y))
	tape_visible = True

def main():
	global tfBuffer,listener
	rospy.init_node('hatch_tf_broadcaster')
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	rospy.Subscriber('lidar_angle', Float64, lidar_callback)
	rospy.Subscriber('vision_pose', PoseXY, vision_callback)
	rospy.loginfo('started hatch transform publisher')
	rospy.Timer(rospy.Duration(.1), hatch_bl_tf)
	rospy.spin()

if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.loginfo('interrupt caught')
