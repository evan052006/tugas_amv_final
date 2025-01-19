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

        cv2.namedWindow("tweakers", cv2.WINDOW_AUTOSIZE)
        cv2.createTrackbar("red_trackbar", "tweakers", 0, 179, self.red_trackbar_oncall)
        cv2.createTrackbar("green_lower_trackbar", "tweakers", 0, 179, self.green_lower_trackbar_oncall)
        cv2.createTrackbar("green_upper_trackbar", "tweakers", 0, 179, self.green_upper_trackbar_oncall)

        def empty():
            pass

        cv2.createTrackbar("ct1", "tweakers", 150, 255, empty)
        cv2.createTrackbar("ct2", "tweakers", 40, 255, empty)
        cv2.createTrackbar("red_thresh", "tweakers", 120, 255, empty)


        red_lower_left_limit = np.array([0,50,50])
        red_upper_left_limit = np.array([10,255,255])
        red_lower_right_limit = np.array([169,50,50])
        red_upper_right_limit = np.array([179,255,255])
        green_lower_limit = np.array([40,50,50])
        green_upper_limit = np.array([80,255,255])



    def red_trackbar_oncall(self, val):
        self.red_lower_left_limit = np.array([0,50,50])
        self.red_upper_left_limit = np.array([val,255,255])
        self.red_lower_right_limit = np.array([179-val,50,50])
        self.red_upper_right_limit = np.array([179,255,255])
    
    def green_lower_trackbar_oncall(self, val):
        self.green_lower_limit = np.array([val,50,50])
    
    def green_upper_trackbar_oncall(self, val):
        self.green_upper_limit = np.array([val,255,255])

    def callback(self, data):
        current_frame = self.bridge.imgmsg_to_cv2(data)
        hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        
        red_mask = \
            cv2.inRange(hsv_frame, self.red_lower_left_limit, self.red_upper_left_limit) \
            | cv2.inRange(hsv_frame, self.red_lower_right_limit, self.red_upper_right_limit)
        red = cv2.bitwise_and(hsv_frame, hsv_frame, mask=red_mask)
        red_grayscale = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)


        ret, thresh1 = cv2.threshold(red_grayscale \
                                     , cv2.getTrackbarPos("red_thresh", "tweakers") \
                                    , 255, cv2.THRESH_BINARY)

        red_grayscale = thresh1
        
        green_mask = cv2.inRange(hsv_frame, self.green_lower_limit, self.green_upper_limit)
        green = cv2.bitwise_and(hsv_frame, hsv_frame, mask=green_mask)
        green_grayscale = cv2.cvtColor(green, cv2.COLOR_BGR2GRAY) 

        cv2.imshow("red", red_grayscale)
        #cv2.imshow("green", green_grayscale)

        cv2.waitKey(1)

        red_count = self.detect_count_from_grayscale(current_frame, red_grayscale)
        #green_count = self.detect_count_from_grayscale(current_frame, green_grayscale)
        green_count = 0

        #if self.current_green_count != self.current_green_count + green_count:
         #   rospy.loginfo(f"green: {self.current_green_count + green_count}")
        #if self.current_red_count != self.current_red_count + red_count:
         #   rospy.loginfo(f"red: {self.current_red_count + red_count}")
        rospy.loginfo("")

        self.publisher.publish(BuoyCount(red_count, green_count))

    def detect_count_from_grayscale(self, original, mask):
        
        blur = cv2.GaussianBlur(mask, (17, 17), 0)
        canny = cv2.Canny(blur\
                          , cv2.getTrackbarPos("ct1", "tweakers") \
                            , cv2.getTrackbarPos("ct2", "tweakers"), 3)
        
        dilated = cv2.dilate(canny, (1, 1), iterations=2)
        (contours, _) = cv2.findContours(
            dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        count = 0
        max_size = -1
        for contour in contours:
            area = cv2.contourArea(contour)
            max_size = max(area, max_size)
            if area >= 200:
                count += 1
        rospy.loginfo(f"contcount: {count}, {max_size}")
        rgb = cv2.cvtColor(original, cv2.COLOR_BGR2RGB)
        cv2.imshow("contour", cv2.drawContours(rgb, contours, -1, (0, 255, 0), 2))
        cv2.waitKey(1)
        return count

if __name__ == '__main__':
    try:
      Utama()
      rospy.spin()
      cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
      pass

