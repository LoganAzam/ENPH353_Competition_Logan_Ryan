#! /usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int32  
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class MountainFollower:
    def __init__(self):
        self.bridge = CvBridge()
        self.active = False

        self.pub_cmd = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
        self.pub_score = rospy.Publisher('/score_tracker', String, queue_size=1)
        self.pub_state = rospy.Publisher('/state_changer', Int32, queue_size=1, latch=True)

        self.image_sub = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.callback, queue_size=3)
        self.state_sub = rospy.Subscriber('/state_changer', Int32, self.state_callback, queue_size=1)

        self.corner_yet_1 = False
        self.corner_yet_2 = False
        self.corner_yet_3 = False

        self.signFound = False

        rospy.sleep(1)

    def state_callback(self, msg):
        # State 6 is for Mountain
        if msg.data == 6:
            if not self.active:
                rospy.loginfo("mountain_PID node activated.")
                self.turn_buffer = rospy.get_time() + 1.0
                self.blue_pix_suppression = rospy.get_time() + 7.0
                self.tunnel_nav = rospy.get_time() + 7.0 + 1.0
                self.corner_nav = rospy.get_time() + 7.0 + 1.0 + 10.5
            self.active = True
        else:
            if self.active:
                rospy.loginfo("mountain_PID node deactivated.")
            self.active = False

    def callback(self, data):
        if not self.active:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return
        
        if self.signFound:
            self.pub_cmd.publish(Twist())

            # Deactivate this node
            self.active = False

            # Switch the global state to sweep
            self.pub_state.publish(-2)
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        h, w, _ = cv_image.shape
        search_top = int(0.5 * h)
        move = Twist()

        lower_shade = np.array([0, 0, 105])
        upper_shade = np.array([140, 255, 150])
        shade_mask = cv2.inRange(hsv, lower_shade, upper_shade)

        lower_white = np.array([26, 0, 122])
        upper_white = np.array([103, 72, 195])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        white_mask_bottom = white_mask[int(0.6 * h):, :]
        white_mask_bottom[:, int(w / 2):w] = 0
        white_pixels = cv2.countNonZero(white_mask)

        # cv2.imshow("Shade Mask", shade_mask)  # Debugging window to visualize the mask
        # cv2.waitKey(1)
        # cv2.imshow("White Mask", white_mask_bottom)  # Debugging window to visualize the mask
        # cv2.waitKey(1)

        shade_moments = cv2.moments(shade_mask)
        white_moments = cv2.moments(white_mask_bottom)

        target_center_shade = int(w / 2)
        target_center_white = int(w / 5)
        if not self.corner_yet_3:
            if rospy.get_time() > self.corner_nav:
                #rospy.loginfo("Corner navigation phase: Adjusting target center for white pixels.")
                target_center_white = int(w / 6)

            p_gain = 65

            if rospy.get_time() < self.turn_buffer:
                # If we're in the buffer period, just keep moving forward
                move.linear.x = 0.1
                move.angular.z = 2.7
            elif shade_moments['m00'] > 0 and rospy.get_time() < self.tunnel_nav:
                cx = int(shade_moments['m10'] / shade_moments['m00'])
                error = cx - target_center_shade
                move.linear.x = 0.6 # Slower for dirt road curves
                move.angular.z = -float(error) / p_gain
            elif white_moments['m00'] > 0 and white_pixels < 22000:
                cx = int(white_moments['m10'] / white_moments['m00'])
                error = cx - target_center_white
                move.linear.x = 0.6 # Slower for dirt road curves
                move.angular.z = -float(error) / 80
            elif white_pixels >= 22000:
                rospy.loginfo("High white pixel count detected.")
                if not self.corner_yet_1:
                    self.corner_buffer = rospy.get_time() + 1.8
                    while rospy.get_time() < self.corner_buffer:
                        rospy.loginfo("Turn 1.")
                        move.linear.x = 0.4 # Slower for dirt road curves
                        move.angular.z = 1.8
                        self.pub_cmd.publish(move)
                    self.corner_buffer = rospy.get_time() + 0.5
                    self.corner_yet_1 = True

                    # self.pub_cmd.publish(Twist())
                    # self.pub_state.publish(9)
                    # # Deactivate this node
                    # self.active = False
                    # return

                elif not self.corner_yet_2 and rospy.get_time() > self.corner_buffer:
                    self.corner_buffer = rospy.get_time() + 1.8
                    while rospy.get_time() < self.corner_buffer:
                        rospy.loginfo("Turn 2.")
                        move.linear.x = 0.4 # Slower for dirt road curves
                        move.angular.z = 1.5
                        self.pub_cmd.publish(move)
                    self.corner_buffer = rospy.get_time() + 1
                    self.corner_yet_2 = True
                elif not self.corner_yet_3 and rospy.get_time() > self.corner_buffer:
                    self.corner_buffer = rospy.get_time() + 1.8
                    while rospy.get_time() < self.corner_buffer:
                        rospy.loginfo("Turn 3.")
                        move.linear.x = 0.0 # Slower for dirt road curves
                        move.angular.z = 1.6
                        self.pub_cmd.publish(move)
                    self.corner_yet_3 = True

                    # self.pub_cmd.publish(Twist())
                    # self.pub_state.publish(9)
                    # # Deactivate this node
                    # self.active = False
                    # return
            else:
                rospy.loginfo("No significant features detected.")
                move.linear.x = 0.6 # Default speed
                move.angular.z = 0.03

        lower_blue = np.array([100, 120, 30])
        upper_blue = np.array([140, 255, 255])

        # 2. Create the mask
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # 3. Define the ROI (Bottom Half)
        h, w, _ = cv_image.shape
        # Slicing from h/2 to the end of the height
        blue_bottom_half = blue_mask[int(0.4 * h):, :]
        #cv2.imshow("Blue Mask", blue_bottom_half)  # Debugging window to visualize the mask
        #cv2.waitKey(1)

        # 4. Count the non-zero (white) pixels
        blue_count = cv2.countNonZero(blue_bottom_half)

        if self.corner_yet_3:
            blue_moments = cv2.moments(blue_bottom_half)
            target_center_blue = int(w / 2)
            if blue_moments['m00'] > 0:
                cx = int(blue_moments['m10'] / blue_moments['m00'])
                error = cx - target_center_blue
                move.linear.x = 0.7 # Slower for dirt road curves
                move.angular.z = -float(error) / 80

        if blue_count > 18000 and rospy.get_time() > self.blue_pix_suppression: # Threshold for confirming the blue sign, with a time buffer to prevent false positives
            self.signFound = True
            rospy.loginfo(f"Blue Sign Confirmed! Switching to Sweep. Count: {blue_count}")
            # STOP the robot
            self.pub_cmd.publish(Twist())

            # Deactivate this node
            self.active = False

            # Switch the global state to sweep
            self.pub_state.publish(0)
            self.signFound = True
            return
        

        self.pub_cmd.publish(move)

if __name__ == '__main__':
    rospy.init_node('mountain_PID_node')
    try:
        follower = MountainFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass