#! /usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int32  
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class DirtroadFollower:
    def __init__(self):
        self.bridge = CvBridge()
        self.active = False  
    
        self.pub_cmd = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
        self.pub_score = rospy.Publisher('/score_tracker', String, queue_size=1)
        self.pub_state = rospy.Publisher('/state_changer', Int32, queue_size=1, latch=True)

        self.image_sub = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.callback, queue_size=3)
        self.state_sub = rospy.Subscriber('/state_changer', Int32, self.state_callback, queue_size=1)

        self.signfound = False
        self.lakeFound = False
        self.turnkey = False

        rospy.sleep(1)
    
    def state_callback(self, msg):
        # State 3 is for Dirt Road
        if msg.data == 3:
            if not self.active:
                rospy.loginfo("dirtroad_PID node activated.")
            self.active = True
            self.blue_pix_suppression_end = rospy.get_time() + 7.0 # Initialize to current time so it's not active at start
            self.blue_pix_suppression_lake = rospy.get_time() + 1.5 # Longer suppression for lake since the blue pixels there are more likely to cause false positives
        else:
            if self.active:
                rospy.loginfo("dirtroad_PID node deactivated.")
            self.active = False

    def callback(self, data):
        if not self.active:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        h, w, _ = cv_image.shape
        search_top = int(0.65 * h)
        move = Twist()

        if self.turnkey:
            turn_time = rospy.get_time() + 1.0 # Time to turn right and find the lake faster, since the sign is on the left side of the road
            while rospy.get_time() < turn_time:
                move.angular.z = -0.75
                self.pub_cmd.publish(move)
                rospy.loginfo("Turning right to equalize")
            self.pub_cmd.publish(Twist())
            self.turnkey = False
            return

        # --- DIRT ROAD MASK ---
        # NOTE: If the dirt road doesn't have a white line, 
        # this white mask will return 0 pixels and the robot will just spin!
        # lower_white = np.array([0, 0, 200])
        # upper_white = np.array([180, 50, 255])
        lower_white = np.array([25, 0, 174])
        upper_white = np.array([65, 76, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)
        #cv2.imshow("Dirt Road Mask", mask) # DEBUG: Show the mask to verify it's working
        #cv2.waitKey(1)  # Add this line to allow OpenCV to process the window events

        lower_water = np.array([92, 0, 109])
        upper_water = np.array([179, 104, 255])
        watermask = cv2.inRange(hsv, lower_water, upper_water)
        watermask[0:int(0.5 * h), :] = 0 # Focus on bottom half
        #cv2.imshow("Water Mask", watermask) # DEBUG: Show the water mask to verify it's working
        #cv2.waitKey(1)  # Add this line to allow OpenCV to process the window events

        if cv2.countNonZero(watermask) > 10000 and self.signfound and rospy.get_time() > self.blue_pix_suppression_end: # If we see a lot of water pixels, we can be pretty confident we're at the lake. The signfound condition is to prevent false positives from puddles in the dirt road.
            rospy.loginfo("Water detected! Pausing PID and switching to Lake PID.")
            self.pub_cmd.publish(Twist())
            self.active = False
            self.lakeFound = True
            self.pub_state.publish(4)
            return

        # ROI: Focus on bottom half
        mask[0:search_top, :] = 0
        if not self.signfound:
            mask[:, int(w/2):w] = 0 # Hugging left
            target_center = w / 4
        else:
            #rospy.loginfo("Sign found, hugging right!")
            mask[:, 0:int(w/2)] = 0 # Hugging right
            target_center = int(3*(w / 4))

        p_gain = 50

        moments = cv2.moments(mask)
        if moments['m00'] > 0 and not self.signfound: # Only do PID if we haven't found the sign yet
            cx = int(moments['m10'] / moments['m00'])
            error = cx - target_center
            move.linear.x = 0.6 # Slower for dirt road curves
            move.angular.z = -float(error) / p_gain
        elif self.signfound and not self.lakeFound: # If we've found the sign but not the lake, we want to turn sharply right to find the lake faster
            rospy.loginfo("Lakefind")
            move.linear.x = 0.35
            move.angular.z = -0.75
        elif moments['m00'] > 0 and self.lakeFound: # Only do PID if we haven't found the sign yet
            cx = int(moments['m10'] / moments['m00'])
            error = cx - target_center
            move.linear.x = 0.6 # Slower for dirt road curves
            move.angular.z = -float(error) / p_gain
        else:
            rospy.loginfo("Initial Line Find")
            move.linear.x = 0.3
            move.angular.z = 0.3

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
        blue_bottom_half = blue_mask[int(h/2):, :]

        # 4. Count the non-zero (white) pixels
        blue_count = cv2.countNonZero(blue_bottom_half)

        # 1. Logic Check: Are we allowed to look for blue? 
        # (Either it's the first sign OR the suppression timer has passed)
        if rospy.get_time() > self.blue_pix_suppression_end:
            
            # 2. Threshold Check: Is there actually a sign in front of us?
            if blue_count > 7000:
                rospy.loginfo(f"Blue Sign Confirmed! Switching to Plate Sweep. Count: {blue_count}")
                # STOP the robot
                self.pub_cmd.publish(Twist())
                turn_time = rospy.get_time() + 1.0 # Time to turn right and find the lake faster, since the sign is on the left side of the road
                move.linear.x = 0.0
                move.angular.z = 0.75
                while rospy.get_time() < turn_time:
                    self.pub_cmd.publish(move)
                self.pub_cmd.publish(Twist())

                # Deactivate this node
                self.active = False
                
                # Switch the global state to Plate Sweep (State 0)
                self.pub_state.publish(0)
                self.signfound = True
                self.turnkey = True


                # 3. EXIT the callback immediately so we don't publish the PID move command below
                return 
            
        if self.lakeFound and blue_count > 9000 and rospy.get_time() > self.blue_pix_suppression_lake:
            rospy.loginfo(f"Blue Sign Confirmed! Switching to Dirtroad. Count: {blue_count}")
            # STOP the robot
            self.pub_cmd.publish(Twist())

            # Deactivate this node
            self.active = False

            # Switch the global state to sweep
            self.pub_state.publish(0)
            return

        lower_magenta = np.array([140, 100, 100])
        upper_magenta = np.array([160, 255, 255])
        
        # Define Magenta HSV range (captures both ends of the hue spectrum)
        magenta_mask = cv2.inRange(hsv, lower_magenta, upper_magenta)

        # Look specifically at the bottom 40% of the screen
        if cv2.countNonZero(magenta_mask[int(0.8*h):, :]) > 1000 and self.signfound: # The signfound condition is to prevent false positives from things like pink flowers on the side of the road
                rospy.loginfo("Magenta line detected! Moving to roadless")
                
                # 1. STOP the robot immediately to avoid collision penalties (-5pts)
                self.pub_cmd.publish(Twist())
                
                # 2. Deactivate this node locally
                self.active = False
                
                # 3. Trigger the next node (State 2 = Motion Tracking)
                self.pub_state.publish(5) 
                return

        self.pub_cmd.publish(move)

if __name__ == '__main__':
    rospy.init_node('dirtroad_PID_node')
    try:
        follower = DirtroadFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass