#!/usr/bin/env python3

# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from tugas_amv_final.msg import BuoyCount
import cv2 # OpenCV library
import numpy as np


class Utama(object):
    def __init__(self):
        rospy.init_node('Utama', anonymous=True)
        rospy.Subscriber('video_frames', Image, self.callback)
        self.publisher = rospy.Publisher('buoy_count', BuoyCount, queue_size=10)
        self.bridge = CvBridge()
        self.current_red_count = 0
        self.current_green_count = 0
    
    def callback(self, data):
        rospy.loginfo("receiving video frame")
        current_frame = self.bridge.imgmsg_to_cv2(data)
        hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        red_lower_left_limit = np.array([10,50,50])
        red_upper_left_limit = np.array([10,255,255])
        red_lower_right_limit = np.array([170,50,50])
        red_upper_right_limit = np.array([180,255,255])
        red_mask = \
            cv2.inRange(hsv_frame, red_lower_left_limit, red_upper_left_limit) \
            | cv2.inRange(hsv_frame, red_lower_right_limit, red_upper_right_limit)

        green_lower_limit = np.array([40,50,50])
        green_upper_limit = np.array([80,255,255])
        green_mask = cv2.inRange(hsv_frame, green_lower_limit, green_upper_limit)

        red_count = self.detect_count_from_grayscale(current_frame, red_mask)
        green_count = self.detect_count_from_grayscale(current_frame, green_mask)

        self.publisher.publish(BuoyCount(red_count, green_count))

    def detect_count_from_grayscale(self, original, mask):
        blur = cv2.GaussianBlur(mask, (11, 11), 0)
        canny = cv2.Canny(blur, 30, 150, 3)
        dilated = cv2.dilate(canny, (1, 1), iterations=0)

        (cnt, hierarchy) = cv2.findContours(
            dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        rgb = cv2.cvtColor(original, cv2.COLOR_BGR2RGB)
        cv2.drawContours(rgb, cnt, -1, (0, 255, 0), 2)
        return len(cnt)

if __name__ == '__main__':
    try:
      Utama()
      rospy.spin()
      cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
      pass

