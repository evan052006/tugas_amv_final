#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

class Movement(object):
	def __init__(self):
		rospy.init_node('Movement', anonymous=True)
		self.buoy_count_subscriber = rospy.Subscriber("buoy_count", Int32, self.callback)
		self.previous_green = 0
		self.previous_red = 0

	def callback(self, data):
		if data.green - self.previous_green >= 3:
			rospy.loginfo("Turning left for 1.5 seconds")
			rospy.sleep(1.5)
			self.previous_green = data.green
		elif data.red - self.previous_red >= 4:
			rospy.loginfo("Turning right for 1.5 seconds")
			rospy.sleep(1.5)
			self.previous_red = data.red
		else:
			rospy.loginfo("Currently moving forward")

if __name__ == '__main__':
	try:
		movement = Movement()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
