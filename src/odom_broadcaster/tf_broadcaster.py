#!/usr/bin/env python
import rospy 
from networktables import NetworkTables
import tf
from math import sin, cos, pi 
from nav_msgs.msg import Odometry
from geomtry_msgs.msg import Point,Pose,Quaternion,Twist,Vector3

sd = None
x,y,theta = 0,0,0
odom_pub = rospy.Publisher('odom',Odometry, queue_size=50)
odom_br = tf.TransformBroadcaster()

def odometry_broadcaster():
        global x,y,theta 

        current_time = rospy.Time.now()

        dx = sd.getNumber('linear_velocity')
        dy = 0
        dtheta = sd.getNumber('angular_velocity')

        dt = 1/10 # 10 Hz update rate bottlenecks the robotcodes 50 Hz rate

        x += dx * cos(theta) * dt
        y += dy * sin(theta) * dt
        theta += dtheta * dt

        theta_quat = tf.transformations.quaternion_from_euler(0,0,theta)

	odom_br.sendTransform((x,y,0),m_euler(0,0,theta),current_time,'base_link','odom')

        odom = Odometry()
        odom.header.stamp = current_time 
        odom.pose.pose = Pose(Point(x,y,0),Quaternion(*odom_quat))
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(dx, dy, 0),Vector3(0,0,dtheta))
        odom_pub.publish(odom)
        
	
def main():
        global sd
	rospy.init_node('odometry_publisher')
	rospy.loginfo('started odom publisher')
        sd = NetworkTables.getTable("SmartDashboard")
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		odometry_broadcaster()
		rate.sleep()

if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.loginfo('interrupt caught')
