#!/usr/bin/env python
import rospy 
import tf

def map_hatch_broadcaster():
	br = tf.TransformBroadcaster()
	br.sendTransform((0,0,0),[0,0,0,1],rospy.Time.now(),'hatch_frame','map')
	
def main():
	rospy.init_node('map_tf_broadcaster')
	rospy.loginfo('started map transform publisher')
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		map_hatch_broadcaster()
		rate.sleep()

if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.loginfo('interrupt caught')
