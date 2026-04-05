#! /usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int32  
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class LakeFollower:
    def __init__(self):
        self.bridge = CvBridge()
        self.active = False  
    
        self.pub_cmd = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
        self.pub_score = rospy.Publisher('/score_tracker', String, queue_size=1)
        self.pub_state = rospy.Publisher('/state_changer', Int32, queue_size=1, latch=True)

        self.image_sub = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.callback, queue_size=3)
        self.state_sub = rospy.Subscriber('/state_changer', Int32, self.state_callback, queue_size=1)

        self.signfound = False

        rospy.sleep(1)
    
    def state_callback(self, msg):
        # State 4 is for Lake
        if msg.data == 4:
            if not self.active:
                rospy.loginfo("lake_PID node activated.")
            self.active = True
            self.blue_pix_suppression_end = rospy.get_time() + 2.0 # Initialize to current time so it's not active at start
        else:
            if self.active:
                rospy.loginfo("lake_PID node deactivated.")
            self.active = False

    def callback(self, data):
        if not self.active:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return

        if self.signfound: 
            rospy.loginfo(f"Switching to right side dirtroad")
                # STOP the robot
            self.pub_cmd.publish(Twist())
                
                # Deactivate this node
            self.active = False
                
                # Switch the global state to Plate Sweep (State 0)
            self.pub_state.publish(3)

                # 3. EXIT the callback immediately so we don't publish the PID move command below
            return 

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        h, w, _ = cv_image.shape
        search_top = int(0.50 * h)
        move = Twist()

        # --- DIRT ROAD MASK ---
        # NOTE: If the dirt road doesn't have a white line, 
        # this white mask will return 0 pixels and the robot will just spin!
        # lower_white = np.array([0, 0, 200])
        # upper_white = np.array([180, 50, 255])
        lower_path = np.array([0, 49, 117])
        upper_path = np.array([105, 255, 255])
        mask = cv2.inRange(hsv, lower_path, upper_path)
        

        # ROI: Focus on bottom half
        mask[0:int(0.65 * h), :] = 0
        #cv2.imshow("Dirt Road Mask", mask) # DEBUG: Show the mask to verify it's working
        #cv2.waitKey(1)  # Add this line to allow OpenCV to process the window events
        target_center = w / 2

        p_gain = 50

        moments = cv2.moments(mask)
        if moments['m00'] > 0 and not self.signfound: # Only do PID if we haven't found the sign yet
            cx = int(moments['m10'] / moments['m00'])
            error = cx - target_center
            move.linear.x = 0.5 # Slower for dirt road curves
            move.angular.z = -float(error) / p_gain
        elif self.signfound:
            move.linear.x = 0.3
            move.angular.z = -0.85
        else:
            move.linear.x = 0.3
            move.angular.z = 0.5

        #self.pub_cmd.publish(move)

        # 1. Define the Blue HSV range
        # Hue: 100-140 (Blue), Saturation: 120-255 (Vibrant), Value: 30-255 (Brightness)
        lower_blue = np.array([100, 120, 30])
        upper_blue = np.array([140, 255, 255])

        # 2. Create the mask
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # 3. Define the ROI (Bottom Half)
        h, w, _ = cv_image.shape
        # Slicing from h/2 to the end of the height
        blue_bottom_half = blue_mask[int(h/2) - int(h/16):int(4*h/5), int(w/2):int(w)]

        # 4. Count the non-zero (white) pixels
        blue_count = cv2.countNonZero(blue_bottom_half)
        #cv2.imshow("Blue Mask", blue_bottom_half) # DEBUG: Show the mask to verify it's working
        #cv2.waitKey(1)  # Add this line to allow OpenCV to process the window events

        # 1. Logic Check: Are we allowed to look for blue? 
        # (Either it's the first sign OR the suppression timer has passed)
        if rospy.get_time() > self.blue_pix_suppression_end:
            
            # 2. Threshold Check: Is there actually a sign in front of us?
            if blue_count > 2000:
                rospy.loginfo(f"Blue Sign Confirmed! Switching to Plate Sweep. Count: {blue_count}")
                # STOP the robot
                self.pub_cmd.publish(Twist())
                
                # Deactivate this node
                self.active = False
                
                # Switch the global state to Plate Sweep (State 0)
                self.pub_state.publish(0)
                self.signfound = True

                # 3. EXIT the callback immediately so we don't publish the PID move command below
                return 
            
        self.pub_cmd.publish(move)

if __name__ == '__main__':
    rospy.init_node('Lake_PID_node')
    try:
        follower = LakeFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass