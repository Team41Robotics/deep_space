#!/usr/bin/env python
import rospy 
from networktables import NetworkTables
import tf
from math import sin, cos, pi 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point,Pose,Quaternion,Twist,Vector3

sd = None
x,y,theta,previous_time = 0,0,0,0
odom_pub = rospy.Publisher('odom',Odometry, queue_size=50)
odom_br = tf.TransformBroadcaster()

def odometry_broadcaster():
        global x,y,theta,previous_time

        current_time = rospy.Time.now()

        dx = sd.getNumber('linear_velocity',0.0)
        dy = 0
        dtheta = sd.getNumber('angular_velocity',0.0)
	theta = sd.getNumber('angle',0.0)

	dt = sd.getNumber('dt',0.0)
	#dt = (current_time-previous_time).secs
        #dt = 1/10 # 10 Hz update rate bottlenecks the robotcodes 50 Hz rate

        x += dx * cos(theta) * dt
        y += dx * sin(theta) * dt
        #theta += dtheta * dt

        theta_quat = tf.transformations.quaternion_from_euler(0,0,theta)

	#rospy.loginfo('x:'+str(x)+'\ty:' + str(y) + '\ttheta:'+str(theta))
	odom_br.sendTransform((x,y,0),theta_quat,current_time,'base_link','odom')

        odom = Odometry()
        odom.header.stamp = current_time 
        odom.pose.pose = Pose(Point(x,y,0),Quaternion(*theta_quat))
        odom.child_frame_id = "base_link"
	odom.header.frame_id = 'odom'
        odom.twist.twist = Twist(Vector3(dx, dy, 0),Vector3(0,0,dtheta))
        odom_pub.publish(odom)
	previous_time = current_time
        
	
def main():
        global sd
	global previous_time
	rospy.init_node('odometry_publisher')
	rospy.loginfo('started odom publisher')
	NetworkTables.initialize(server='10.0.41.2')
        sd = NetworkTables.getTable("SmartDashboard")
	rate = rospy.Rate(10)
	previous_time = rospy.Time.now()
	while not rospy.is_shutdown():
		odometry_broadcaster()
		rate.sleep()

if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.loginfo('interrupt caught')
