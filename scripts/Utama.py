#!/usr/bin/env python3
import rospy 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
from tugas_amv_final.msg import BuoyCount
import cv2 
import numpy as np

class Utama(object):
    def __init__(self):
        rospy.init_node('Utama', anonymous=True)
        rospy.Subscriber('video_frames', Image, self.callback)
        self.publisher = rospy.Publisher('buoy_count', BuoyCount, queue_size=10)
        self.bridge = CvBridge()
        self.current_red_count = 0
        self.current_green_count = 0

        self.red_lower_left_limit = np.array([10,50,50])
        self.red_upper_left_limit = np.array([10,255,255])
        self.red_lower_right_limit = np.array([170,50,50])
        self.red_upper_right_limit = np.array([180,255,255])

        self.green_lower_limit = np.array([40,50,50])
        self.green_upper_limit = np.array([80,255,255])

        cv2.createTrackbar("red_trackbar", "red", 0, 179, self.red_trackbar_oncall)
        cv2.createTrackbar("green_lower_trackbar", "red", 0, 179, self.green_lower_trackbar_oncall)
        cv2.createTrackbar("green_upper_trackbar", "green", 0, 179, self.green_upper_trackbar_oncall)

        #red_lower_left_limit = np.array([10,50,50])
        #red_upper_left_limit = np.array([10,255,255])
        #red_lower_right_limit = np.array([170,50,50])
        #red_upper_right_limit = np.array([180,255,255])

        #green_lower_limit = np.array([40,50,50])
        #green_upper_limit = np.array([80,255,255])

    def red_trackbar_oncall(self, val):
        self.red_lower_left_limit = np.array([val,50,50])
        self.red_upper_left_limit = np.array([val,255,255])
        self.red_lower_right_limit = np.array([179-val,50,50])
        self.red_upper_right_limit = np.array([179-val,255,255])
    
    def green_lower_trackbar_oncall(self, val):
        self.green_lower_limit = val
    
    def green_upper_trackbar_oncall(self, val):
        self.green_upper_limit = val

    def callback(self, data):
        current_frame = self.bridge.imgmsg_to_cv2(data)
        hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        
        red_mask = \
            cv2.inRange(hsv_frame, self.red_lower_left_limit, self.red_upper_left_limit) \
            | cv2.inRange(hsv_frame, self.red_lower_right_limit, self.red_upper_right_limit)
        red = cv2.bitwise_and(hsv_frame, hsv_frame, mask=red_mask)
        red_grayscale = cv2.COLOR_BGR2GRAY(red)
        
        green_mask = cv2.inRange(hsv_frame, self.green_lower_limit, self.green_upper_limit)
        green = cv2.bitwise_and(hsv_frame, hsv_frame, mask=green_mask)
        green_grayscale = cv2.COLOR_BGR2GRAY(green)

        cv2.imshow("red", red_grayscale)
        cv2.imshow("green", green_grayscale)

        red_count = self.detect_count_from_grayscale(current_frame, red_grayscale)
        green_count = self.detect_count_from_grayscale(current_frame, green_grayscale)

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

