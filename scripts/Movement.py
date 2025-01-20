#!/usr/bin/env python
import rospy
from tugas_amv_final.msg import BuoyCount

class Movement(object):
	def __init__(self):
		rospy.init_node('Movement', anonymous=True)
		self.buoy_count_subscriber = rospy.Subscriber("buoy_count", BuoyCount, self.callback)
		self.previous_green = 0
		self.previous_red = 0
		self.hasnt_printed_moving_foward = True

	def callback(self, data):
		times_to_turn_left = (data.green - self.previous_green) // 3
		times_to_turn_right = (data.red - self.previous_red) // 4
		if times_to_turn_left:
			for _ in range(times_to_turn_left):
				rospy.loginfo("Turning left for 1.5 seconds")
				rospy.sleep(1.5)
				rospy.loginfo("Finished turning left")
			self.previous_green = data.green
			self.hasnt_printed_moving_foward = True
		elif times_to_turn_right:
			for _ in range(times_to_turn_right):
				rospy.loginfo("Turning right for 1.5 seconds")
				rospy.sleep(1.5)
				rospy.loginfo("Finished turning right")
			self.previous_red = data.red
			self.hasnt_printed_moving_foward = True
		if self.hasnt_printed_moving_foward:
			rospy.loginfo("Currently moving forward")
			self.hasnt_printed_moving_foward = False

if __name__ == '__main__':
	try:
		movement = Movement()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
