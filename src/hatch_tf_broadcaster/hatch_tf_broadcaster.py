#!/usr/bin/env python
import rospy 
import tf 
from deep_space.msg import PoseXY 
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped

pos_x, pos_y, theta = 0, 0, 0

def publish():
	br = tf.TransformBroadcaster()
	listener = tf.TransformListener()
	l2bl = listener.asMatrix('/base_lidar','/base_link')
	c2bl = listener.asMatrix('/base_lidar','/base_link')
	h2bl = TransformStamped()
	h2bl.header.stamp = rospy.Time.now()
	h2bl.header.frame_id = 'hatch_frame'
	h2bl.child_frame_id = 'base_link'
	h2bl_matrix = c2bl*l2bl.I
	h2bl.transform.translation.x = h2bl_matrix[0,3]
	h2bl.transform.translation.y = h2bl_matrix[1,3]
	h2bl.transform.translation.z = h2bl_matrix[2,3]
	h2bl.transform.rotation.x = tf.quaternion_from_matrix(h2bl_matrix)[0]
	h2bl.transform.rotation.y = tf.quaternion_from_matrix(h2bl_matrix)[1]
	h2bl.transform.rotation.z = tf.quaternion_from_matrix(h2bl_matrix)[2]
	h2bl.transform.rotation.w = tf.quaternion_from_matrix(h2bl_matrix)[3]
	print(h2bl)
	#br.sendTransform(h2bl)

def hatch_camera_tf():
	br = tf.TransformBroadcaster()
	h2cam = TransformStamped()
	h2cam.header.stamp = rospy.Time.now()
	h2cam.header.frame_id = 'hatch_frame'
	h2cam.child_frame_id = 'base_camera'
	h2cam.transform.translation.x = pos_x 
	h2cam.transform.translation.y = -pos_y
	h2cam.transform.translation.z = 0  
	quat = tf.transformations.quaternion_from_euler(0, 0, theta)
	h2cam.transform.rotation.x = quat[0]
	h2cam.transform.rotation.y = quat[1]
	h2cam.transform.rotation.z = quat[2]
	h2cam.transform.rotation.w = quat[3]
	br.sendTransform(h2cam.transform.translation,h2cam.transform.rotation,h2cam.header.stamp,h2cam.child_frame_id,h2cam.header.frame_id)

def lidar_callback(msg): 
	global theta 
	theta = msg.data
	hatch_camera_tf()
	publish()

def vision_callback(msg):
	global pos_x, pos_y
	pos_x = msg.x
	pos_y = msg.y 
	hatch_camera_tf() 
	publish()

def main():
	rospy.init_node('hatch_tf_broadcaster')
	rospy.Subscriber('lidar_angle', Float64, lidar_callback)
	rospy.Subscriber('vision_pose', PoseXY, vision_callback)
	rospy.loginfo('started hatch transform publisher')
	rospy.spin()

if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.loginfo('interrupt caught')
