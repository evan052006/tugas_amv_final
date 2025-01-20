#!/usr/bin/env python3
import rospy 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
from tugas_amv_final.msg import BuoyCount
import cv2 
import numpy as np

class Debouncer(object):
    def __init__(self, debounce_delay, last_count = 0, current_count = 0):
        self.last_count = last_count
        self.debounce_delay = debounce_delay
        self.debounce_timer = 0
        self.current_count = current_count
    
    def debounce(self, count):
        if count == self.last_count:
            self.debounce_timer += 1
        else:
            self.debounce_timer = 0
        self.last_count = count
        if self.debounce_timer >= self.debounce_delay and self.current_count != count:
            self.current_count = count
            self.debounce_timer = 0
            return count
        else:
            return self.current_count

class Utama(object):
    def __init__(self):
        rospy.init_node('Utama', anonymous=True)
        rospy.Subscriber('video_frames', Image, self.callback)
        self.publisher = rospy.Publisher('buoy_count', BuoyCount, queue_size=10)
        self.bridge = CvBridge()
        self.current_red_count = 0
        self.current_green_count = 0
        self.prev_red_count = 0
        self.prev_green_count = 0
        self.green_debouncer = Debouncer(5)
        self.red_debouncer = Debouncer(5)

        self.red_lower_left_limit = np.array([10,50,50])
        self.red_upper_left_limit = np.array([10,255,255])
        self.red_lower_right_limit = np.array([170,50,50])
        self.red_upper_right_limit = np.array([180,255,255])

        self.green_lower_limit = np.array([40,50,50])
        self.green_upper_limit = np.array([80,255,255])

        cv2.namedWindow("parameter", cv2.WINDOW_AUTOSIZE)

        def empty(nothing):
            pass

        cv2.createTrackbar("red_range", "parameter", 10, 179, empty)
        cv2.createTrackbar("green_lower_range", "parameter", 40, 179, empty)
        cv2.createTrackbar("green_upper_range", "parameter", 80, 179, empty)
        cv2.createTrackbar("red_ct1", "parameter", 150, 255, empty)
        cv2.createTrackbar("red_ct2", "parameter", 40, 255, empty)
        cv2.createTrackbar("green_ct1", "parameter", 150, 255, empty)
        cv2.createTrackbar("green_ct2", "parameter", 40, 255, empty)
        cv2.createTrackbar("red_thresh", "parameter", 120, 255, empty)
        cv2.createTrackbar("green_thresh", "parameter", 0, 255, empty)

    def callback(self, data):
        current_frame = self.bridge.imgmsg_to_cv2(data)
        hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        
        val = cv2.getTrackbarPos("red_range", "parameter")
        left_lower = np.array([0,50,50])
        left_upper = np.array([val,255,255])
        right_lower = np.array([179-val,50,50])
        right_upper = np.array([179,255,255])
        red_mask = \
            cv2.inRange(hsv_frame, left_lower, left_upper) \
            | cv2.inRange(hsv_frame, right_lower, right_upper)
        red = cv2.bitwise_and(hsv_frame, hsv_frame, mask=red_mask)
        red_grayscale = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
        val_thresh = cv2.getTrackbarPos("red_thresh", "parameter")
        _, red_thresh = cv2.threshold(red_grayscale, val_thresh, 255, cv2.THRESH_BINARY)

        val1 = cv2.getTrackbarPos("green_lower_range", "parameter")
        val2 = cv2.getTrackbarPos("green_upper_range", "parameter")
        lower = np.array([val1,50,50])
        upper = np.array([val2,255,255])
        green_mask = cv2.inRange(hsv_frame, lower, upper)
        green = cv2.bitwise_and(hsv_frame, hsv_frame, mask=green_mask)
        green_grayscale = cv2.cvtColor(green, cv2.COLOR_BGR2GRAY)
        val_thresh = cv2.getTrackbarPos("green_thresh", "parameter")
        _, green_thresh = cv2.threshold(green_grayscale, val_thresh, 255, cv2.THRESH_BINARY)

        red_ct1 = cv2.getTrackbarPos("red_ct1", "parameter")
        red_ct2 = cv2.getTrackbarPos("red_ct2", "parameter")
        green_ct1 = cv2.getTrackbarPos("green_ct1", "parameter")
        green_ct2 = cv2.getTrackbarPos("green_ct2", "parameter")

        red_count = self.detect_count_from_grayscale(current_frame, red_thresh, 0, red_ct1, red_ct2)
        green_count = self.detect_count_from_grayscale(current_frame, green_thresh, 1, green_ct1, green_ct2)

        red_count = self.red_debouncer.debounce(red_count)
        green_count = self.green_debouncer.debounce(green_count)

        if self.prev_red_count < red_count:
            self.current_red_count += red_count - self.prev_red_count
            
        if self.prev_green_count < green_count:
            self.current_green_count += green_count - self.prev_green_count
        
        self.prev_red_count = red_count
        self.prev_green_count = green_count

        rospy.loginfo(f"total_red: {self.current_red_count} total_green: {self.current_green_count}")
        rospy.loginfo(f"current_red: {self.red_debouncer.current_count} current_green: {self.green_debouncer.current_count}")

        self.publisher.publish(BuoyCount(self.current_red_count, self.current_green_count))

    def detect_count_from_grayscale(self, original, mask, t, ct1, ct2):
        blur = cv2.GaussianBlur(mask, (25, 25), 0)
        canny = cv2.Canny(blur, ct1, ct2, 3)
        dilated = cv2.dilate(canny, (1, 1), iterations=1)
        (contours, _) = cv2.findContours(
            dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        count = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= 100:
                count += 1
        rospy.loginfo(f"contcount: {count}, totalcont: {len(contours)}")
        rgb = cv2.cvtColor(original, cv2.COLOR_BGR2RGB)
        cv2.imshow("green" if t else "red", cv2.drawContours(rgb, contours, -1, (0, 255, 0), 2))
        cv2.waitKey(1)
        return count

if __name__ == '__main__':
    try:
      Utama()
      rospy.spin()
      cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
      pass

