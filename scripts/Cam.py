#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

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
					#cv2.imshow("Camera", frame)
					cv2.waitKey(1)
					self.publisher.publish(self.bridge.cv2_to_imgmsg(frame))
				self.rate.sleep()

if __name__ == '__main__':
	try:
		cam = Cam()
		cam.publish_message()
		cv2.destroyAllWindows()
	except rospy.ROSInterruptException:
		pass