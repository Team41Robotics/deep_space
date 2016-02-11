#!/usr/bin/env python
import rospy 
from networktables import NetworkTables
from geometry_msgs.msg import PoseStamped

sd = None
target_pub = rospy.Publisher('move_base_simple/goal',PoseStamped, queue_size=50)

def send_move_base_goal():

	send_goal = sd.getBoolean('send_nav_goal',False)
	
	if send_goal:
		goal = PoseStamped()
        	goal.header.stamp = rospy.Time.now()
		goal.header.frame_id = 'hatch_frame'
        	goal.pose.pose = Pose(Point(0,0,0),Quaternion(tf.transformations.quaternion_from_euler(0,0,0))
)
        	target_pub.publish(odom)
	else:
		
	
def main():
        global sd
	rospy.init_node('move_base_goal')
	rospy.loginfo('started move base goal')
	NetworkTables.initialize(server='10.0.41.2')
        sd = NetworkTables.getTable("SmartDashboard")
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		send_move_base_goal()
		rate.sleep()

if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.loginfo('interrupt caught')
