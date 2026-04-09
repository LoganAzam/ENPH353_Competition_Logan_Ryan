#! /usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from collections import deque # Necessary for sliding window

class MotionDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.active = False
        self.reference = None
        
        # Logic thresholds
        self.min_pixels = 300
        self.still_frames = 8
        self.moving_frames = 5
        self.timeout = 750 
        
        # Sliding Window for Yoda Mode
        self.window_size = 14
        self.motion_history = deque(maxlen=self.window_size)
        
        # State counters
        self.still_counter = 0
        self.motion_counter = 0
        self.frame_count = 0
        self.has_seen_motion = False
        self.active_count = 0  # should be 0

        # Publishers
        self.pub_cmd = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
        self.pub_state = rospy.Publisher('/state_changer', Int32, queue_size=1)
        
        # Subscribers
        self.image_sub = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.callback)
        self.state_sub = rospy.Subscriber('/state_changer', Int32, self.state_callback)

        rospy.sleep(1)

    def reset(self):
        """Resets the detection state for a new crosswalk."""
        self.reference = None
        self.still_counter = 0
        self.motion_counter = 0
        self.has_seen_motion = False
        self.frame_count = 0
        self.motion_history.clear() # Clear history for new activation

    def state_callback(self, msg):
        self.active = (msg.data == 2)
        if self.active:
            rospy.loginfo("MotionDetector activated.")
            self.active_count += 1
            
            # Per-crosswalk logic based on encounter order
            if self.active_count == 1:
                rospy.loginfo("Pedestrian Mode")
                self.timeout = 150
                self.min_pixels = 1000
            elif self.active_count == 2:
                rospy.loginfo("Truck Mode")
                self.timeout = 750
                self.min_pixels = 200
            else:
                rospy.loginfo("Yoda Mode (Sliding Window Active)")
                self.timeout = 1500
                self.min_pixels = 30
                self.still_frames = 15
            
            self.reset()

    def callback(self, data):
        if not self.active:
            return

        self.frame_count += 1
        
        if self.frame_count > self.timeout:
            rospy.logwarn("Timeout reached.")
            self.pub_state.publish(1)
            if self.active_count == 3:
                self.pub_state.publish(5)
            self.active = False
            return

        try:
            curr_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)
            curr_gray = cv2.GaussianBlur(curr_gray, (21, 21), 0)
        except CvBridgeError:
            return

        if self.reference is None:
            self.reference = curr_gray
            return

        # 2. Motion Detection
        diff = cv2.absdiff(self.reference, curr_gray)
        _, thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)
        
        kernel = np.ones((5, 5), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        
        h, w = thresh.shape
        roi = thresh[int(h/4):, int(w/4):int(3*w/4)]
        motion_pixels = cv2.countNonZero(roi)

        # 3. Decision Logic
        has_motion = motion_pixels > self.min_pixels
        
        # Add current frame status to the sliding window
        self.motion_history.append(1 if has_motion else 0)

        if has_motion:
            self.motion_counter += 1
            self.still_counter = 0
            self.reference = curr_gray 
            
            # Standard consecutive check for Pedestrian/Truck
            if not self.has_seen_motion and self.active_count < 3:
                if self.motion_counter >= self.moving_frames:
                    self.has_seen_motion = True
                    rospy.loginfo("Motion confirmed (Consecutive).")
            
            # Window-based check for Yoda (active_count >= 3)
            elif not self.has_seen_motion and self.active_count >= 3:
                if sum(self.motion_history) >= 8:
                    self.has_seen_motion = True
                    rospy.loginfo("Yoda confirmed (8 of 14 window).")
        else:
            self.motion_counter = 0
            self.still_counter += 1
            if self.still_counter % 20 == 0:
                self.reference = curr_gray

        # 4. Success Condition
        if self.has_seen_motion and self.still_counter >= self.still_frames:
            rospy.loginfo("Clear - Returning to Road PID.")
            self.pub_state.publish(1)
            if self.active_count == 3:
                self.pub_state.publish(5)
            self.active = False

        # cv2.imshow("Motion ROI", roi)
        # cv2.waitKey(1)
        # rospy.loginfo(f"Frame: {self.frame_count}, Motion Pixels: {motion_pixels}, Still Counter: {self.still_counter}, Motion Counter: {self.motion_counter}")

if __name__ == '__main__':
    rospy.init_node('motion_detector')
    try:
        detector = MotionDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass