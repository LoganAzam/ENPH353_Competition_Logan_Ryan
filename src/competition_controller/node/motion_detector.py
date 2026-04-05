#! /usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class MotionDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.active = False
        self.reference = None
        
        # State Tracking
        self.last_state = 1        # Default source state
        self.return_state = 1      # Where we actually go back to
        self.state_1_count = 0     # To distinguish between crosswalk 1 and 2
        
        # Logic thresholds (will be overwritten in callback)
        self.min_pixels = 300
        self.still_frames = 8
        self.moving_frames = 5
        self.timeout = 750
        
        # Internal Counters
        self.still_counter = 0
        self.motion_counter = 0
        self.frame_count = 0
        self.has_seen_motion = False

        # ROS Comm
        self.pub_cmd = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
        self.pub_state = rospy.Publisher('/state_changer', Int32, queue_size=1)
        self.image_sub = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.callback)
        self.state_sub = rospy.Subscriber('/state_changer', Int32, self.state_callback)

        rospy.sleep(1) 

    def reset(self):
        self.reference = None
        self.still_counter = 0
        self.motion_counter = 0
        self.has_seen_motion = False
        self.frame_count = 0

    def state_callback(self, msg):
        # Activation Logic
        if msg.data == 2:
            if not self.active:
                rospy.loginfo(f"MotionDetector activated! Origin State: {self.last_state}")
                self.active = True
                self.return_state = self.last_state # Lock in the return address
                
                # Assign Thresholds based on source
                if self.last_state == 1:
                    self.state_1_count += 1
                    if self.state_1_count == 1:
                        # First Crosswalk (Road)
                        self.timeout = 150
                        self.min_pixels = 1000
                    else:
                        # Truck (Road)
                        self.timeout = 750
                        self.min_pixels = 200
                elif self.last_state == 3:
                    # Third Crosswalk (Dirt Road)
                    self.timeout = 500
                    self.min_pixels = 30

                self.reset()
        else:
            # If the state is NOT 2, we update last_state so we know where 
            # the next call comes from.
            self.active = False
            self.last_state = msg.data

    def callback(self, data):
        if not self.active:
            return

        self.frame_count += 1
        
        # 1. Safety Timeout: Proceed back to origin if stuck
        if self.frame_count > self.timeout:
            rospy.logwarn(f"Timeout! Returning to state {self.return_state}")
            self.exit_node()
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

        # 2. Motion Detection Logic
        diff = cv2.absdiff(self.reference, curr_gray)
        _, thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)
        
        # Morphology
        kernel = np.ones((5, 5), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        
        # Focus on ROI
        h, w = thresh.shape
        roi = thresh[int(h/4):, int(w/4):int(3*w/4)]
        motion_pixels = cv2.countNonZero(roi)

        cv2.imshow("Motion ROI", roi)
        cv2.waitKey(1) 

        # 3. Success Conditions
        has_motion = motion_pixels > self.min_pixels

        if has_motion:
            self.motion_counter += 1
            self.still_counter = 0
            self.reference = curr_gray 
            
            if self.motion_counter >= self.moving_frames and not self.has_seen_motion:
                self.has_seen_motion = True
                rospy.loginfo("Pedestrian detected.")
        else:
            self.motion_counter = 0
            self.still_counter += 1
            if self.still_counter % 20 == 0:
                self.reference = curr_gray

        # 4. Exit Condition
        if self.has_seen_motion and self.still_counter >= self.still_frames:
            rospy.loginfo(f"Clear - Returning to state {self.return_state}")
            self.exit_node()

    def exit_node(self):
        """Helper to cleanup and publish state change."""
        self.pub_state.publish(self.return_state)
        self.active = False
        # Optional: cv2.destroyWindow("Motion ROI")

    

if __name__ == '__main__':
    rospy.init_node('motion_detector')
    try:
        detector = MotionDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass