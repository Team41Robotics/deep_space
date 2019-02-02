import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg  import Image
from std_msgs.msg import String

# If the break with num_loops is enabled,
# This file must be executed after rviz is launched

url = "peter.PNG"
img = cv2.imread(url)
img_msg = CvBridge().cv2_to_imgmsg(img) 

def send():
        pub = rospy.Publisher('img', Image, queue_size=10)
        rospy.init_node('imgNode', anonymous=True)
        rate = rospy.Rate(10)
	num_loops = 1
	send_once = True
        while not rospy.is_shutdown():
		if not send_once:
			rospy.loginfo("Publishing to /img @" + str(rospy.get_time()))
		pub.publish(img_msg)
		rate.sleep()

		# This stops the image from being written more than once
		# RViz receives one image after this loops 5 times
		if send_once:
			if num_loops >= 5: break
			num_loops += 1
	rospy.loginfo(url + " published to /img")

if __name__ == '__main__':
        try:
                send()
        except rospy.ROSInterruptException:
                pass



