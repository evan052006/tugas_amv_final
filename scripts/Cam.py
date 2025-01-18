#!/usr/bin/env python3
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

class Cam(object):
	def __init__(self):
		rospy.init_node('Cam', anonymous=True)
		self.publisher = rospy.Publisher('video_frames', Image, queue_size=10)
		self.rate = rospy.Rate(10)
		self.video_capture = cv2.VideoCapture(0)
		self.bridge = CvBridge()
	def publish_message(self):
		while not rospy.is_shutdown():
				ret, frame = self.video_capture.read()
				if ret == True:
					rospy.loginfo('publishing video frame')
					self.publisher.publish(self.bridge.cv2_to_imgmsg(frame))
				self.rate.sleep()

if __name__ == '__main__':
	try:
		cam = Cam()
		cam.publish_message()
	except rospy.ROSInterruptException:
		pass