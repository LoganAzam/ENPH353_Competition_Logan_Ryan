#! /usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int32  
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class RoadFollower:
    def __init__(self):
        """
        Initializes the RoadFollower node.
        Sets up CV Bridge, Publishers, and Subscribers.
        """
        self.bridge = CvBridge()
        self.active = False  # The node starts in an inactive state
    
        # Mandatory publishers for the competition 
        self.pub_cmd = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
        self.pub_score = rospy.Publisher('/score_tracker', String, queue_size=1)
        self.pub_state = rospy.Publisher('/state_changer', Int32, queue_size=1, latch=True)

        # Correct Topic: Based on your XML <cameraName> and <imageTopicName>
        self.image_sub = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.callback, queue_size=3)
        
        # Subscriber to manage the modular state of the robot
        self.state_sub = rospy.Subscriber('/state_changer', Int32, self.state_callback, queue_size=1)

        self.redline_detected = False
        self.blue_init = True
        self.roundabout = False
        self.signcount = 0

        # Mandatory 1-second delay for ROS Master registration 
        rospy.sleep(1)

    def state_callback(self, msg):
        """
        Updates the active state of the node.
        1 = road_PID active
        0 = road_PID inactive (e.g., plate_sweep mode)
        """
        if msg.data == 1:
            if not self.active:
                rospy.loginfo("road_PID node activated.")
            self.active = True
            self.red_line_suppression_end = rospy.get_time() + 3.0
            self.roundabout_hug_entry = rospy.get_time() + 7.6
            self.roundabout_hug_end = rospy.get_time() + 25.60
            if self.signcount == 3:
                self.blue_pix_suppression_end = rospy.get_time() + 7.0
            else:
                self.blue_pix_suppression_end = rospy.get_time() + 2.0
        else:
            if self.active:
                rospy.loginfo("road_PID node deactivated.")
            self.active = False
 
    def callback(self, data):
        """
        Processes camera feed to follow the grey road using HSV masking and PID.
        """
        # Only execute driving logic if this node is currently active
        if not self.active:
            return

        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        h, w, _ = cv_image.shape
        search_top = int(0.65 * h)
        move = Twist()

        # --- MODE SWITCH: NORMAL VS ROUNDABOUT ---
        if self.signcount == 3 and rospy.get_time() > self.roundabout_hug_entry and rospy.get_time() < self.roundabout_hug_end:
            # 1. Detect WHITE lines specifically
            rospy.loginfo("Roundabout Mode: Looking for WHITE lines only.")
            lower_white = np.array([0, 0, 200])
            upper_white = np.array([180, 50, 255])
            mask = cv2.inRange(hsv, lower_white, upper_white)
            
            # 2. ROI: Focus ONLY on the bottom-left quadrant
            # This prevents picking up the "inner" circle line on the right
            mask[0:search_top, :] = 0
            mask[:, int(w/2):w] = 0 # Black out the right half of the screen
            
            target_center = w / 4 # We want the line to stay in the center of our left-view
            p_gain = 50 # You might need more sensitivity for single lines
        else:
            # 1. Detect GREY road mass
            rospy.loginfo("Normal Mode: Looking for GREY road mass.")
            lower_grey = np.array([0, 0, 50]) 
            upper_grey = np.array([180, 50, 200])
            mask = cv2.inRange(hsv, lower_grey, upper_grey)
            mask[0:search_top, 0:w] = 0
            
            target_center = w / 2 # Center of the road
            p_gain = 65

        # 3. PID Calculation
        moments = cv2.moments(mask)
        if moments['m00'] > 0:
            cx = int(moments['m10'] / moments['m00'])
            error = cx - target_center
            
            # Linear speed logic
            if self.redline_detected and rospy.get_time() < self.red_line_suppression_end:
                move.linear.x = 0.8
                move.angular.z = -float(error) / p_gain
            elif self.signcount == 3 and rospy.get_time() > self.roundabout_hug_entry and rospy.get_time() < self.roundabout_hug_end:
                move.linear.x = 0.8
                move.angular.z = -float(error) / 60
            else:
                move.linear.x = 0.5
                move.angular.z = -float(error) / p_gain

        else:
            # Search behavior
            move.linear.x = 0.05
            move.angular.z = 1

        # Inside the RoadFollower callback
        # Define Red HSV range (captures both ends of the hue spectrum)
        red_mask = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255])) | \
                cv2.inRange(hsv, np.array([170, 100, 100]), np.array([180, 255, 255]))

        # Look specifically at the bottom 40% of the screen
        if rospy.get_time() > self.red_line_suppression_end:
            if cv2.countNonZero(red_mask[int(0.8*h):, :]) > 1000:
                rospy.loginfo("Red line detected! Pausing PID and switching to Motion Tracking.")
                
                # 1. STOP the robot immediately to avoid collision penalties (-5pts)
                self.pub_cmd.publish(Twist())
                
                # 2. Deactivate this node locally
                self.active = False
                
                # 3. Trigger the next node (State 2 = Motion Tracking)
                self.pub_state.publish(2) 
                self.redline_detected = True
                return

        # 1. Define the Blue HSV range
        # Hue: 100-140 (Blue), Saturation: 120-255 (Vibrant), Value: 30-255 (Brightness)
        lower_blue = np.array([100, 120, 30])
        upper_blue = np.array([140, 255, 255])

        # 2. Create the mask
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # 3. Define the ROI (Bottom Half)
        h, w, _ = cv_image.shape
        # Slicing from h/2 to the end of the height
        if self.signcount == 2:
            blue_bottom_half = blue_mask[int(h/2):, 0:int(2*w/3)]
        else:
            blue_bottom_half = blue_mask[int(h/2):, :]

        # 4. Count the non-zero (white) pixels
        blue_count = cv2.countNonZero(blue_bottom_half)

        # 1. Logic Check: Are we allowed to look for blue? 
        # (Either it's the first sign OR the suppression timer has passed)
        if self.blue_init or rospy.get_time() > self.blue_pix_suppression_end:
            
            # 2. Threshold Check: Is there actually a sign in front of us?
            if blue_count > 3500:
                rospy.loginfo(f"Blue Sign Confirmed! Switching to Plate Sweep. Count: {blue_count}")
                self.redline_detected = False # Reset red line flag for next time
                self.signcount += 1
                # STOP the robot
                self.pub_cmd.publish(Twist())
                
                # Deactivate this node
                self.active = False
                self.blue_init = False # The "First time" pass is now used up
                
                # Switch the global state to Plate Sweep (State 0)
                self.pub_state.publish(0)
                
                # 3. EXIT the callback immediately so we don't publish the PID move command below
                return 
        
        lower_magenta = np.array([140, 100, 100])
        upper_magenta = np.array([160, 255, 255])
        
        # Define Magenta HSV range (captures both ends of the hue spectrum)
        magenta_mask = cv2.inRange(hsv, lower_magenta, upper_magenta)

        # Look specifically at the bottom 40% of the screen
        if cv2.countNonZero(magenta_mask[int(0.8*h):, :]) > 1000:
                rospy.loginfo("Magenta line detected! Moving to dirtroad")
                
                # 1. STOP the robot immediately to avoid collision penalties (-5pts)
                self.pub_cmd.publish(Twist())
                
                # 2. Deactivate this node locally
                self.active = False
                
                # 3. Trigger the next node (State 2 = Motion Tracking)
                self.pub_state.publish(3) 
                return

        # If we didn't switch states, publish the normal PID driving command
        self.pub_cmd.publish(move)

        # Publish movement command to the simulation
        self.pub_cmd.publish(move)

        # Optional Debug Window (comment out for final run to save RTF)
        #cv2.imshow("Grey Road Mask", mask)
        #cv2.waitKey(1)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('road_PID_node')
    
    try:
        follower = RoadFollower()
        # Keep the node running until interrupted
        rospy.spin()
    except rospy.ROSInterruptException:
        pass