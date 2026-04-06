#! /usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int32  
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class RoadlessFollower:
    def __init__(self):
        self.bridge = CvBridge()
        self.active = False

        self.pub_cmd = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
        self.pub_score = rospy.Publisher('/score_tracker', String, queue_size=1)
        self.pub_state = rospy.Publisher('/state_changer', Int32, queue_size=1, latch=True)

        self.image_sub = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.callback, queue_size=3)
        self.state_sub = rospy.Subscriber('/state_changer', Int32, self.state_callback, queue_size=1)

        self.signFound = False

        rospy.sleep(1)

    def state_callback(self, msg):
        # State 5 is for Roadless
        if msg.data == 5:
            if not self.active:
                rospy.loginfo("roadless_PID node activated.")
                self.straight_line_buffer = rospy.get_time() + 5.5
                self.turn_buffer = rospy.get_time() + 5.5 + 0.6
                self.run = rospy.get_time() + 5.5 + 0.5 + 4.5
                self.blue_pix_suppression = rospy.get_time() + 5.0
            self.active = True
        else:
            if self.active:
                rospy.loginfo("roadless_PID node deactivated.")
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

        lower_magenta = np.array([140, 100, 100])
        upper_magenta = np.array([160, 255, 255])
        
        # Define Magenta HSV range (captures both ends of the hue spectrum)
        magenta_mask = cv2.inRange(hsv, lower_magenta, upper_magenta)
        # cv2.imshow("Magenta Mask", magenta_mask)  # Debugging window to visualize the mask
        # cv2.waitKey(1)

        moments = cv2.moments(magenta_mask)

        target_center = w / 2
        p_gain = 65

        if rospy.get_time() < self.straight_line_buffer and not self.signFound:
            # If we're in the buffer period, just keep moving forward
            move.linear.x = 1
            move.angular.z = -0.1
        elif rospy.get_time() < self.turn_buffer and not self.signFound:
            # If we're in the buffer period, just keep moving forward
            move.linear.x = 0
            move.angular.z = 5.5
        elif rospy.get_time() < self.run and not self.signFound:
            # If we're in the buffer period, just keep moving forward
            move.linear.x = 1
            move.angular.z = -0.2
        elif moments['m00'] > 0:
            cx = int(moments['m10'] / moments['m00'])
            error = cx - target_center
            move.linear.x = 0.8 # Slower for dirt road curves
            move.angular.z = -float(error) / p_gain

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

        if blue_count > 5000 and rospy.get_time() > self.blue_pix_suppression: # Threshold for confirming the blue sign, with a time buffer to prevent false positives
            self.signFound = True
            rospy.loginfo(f"Blue Sign Confirmed! Switching to Sweep. Count: {blue_count}")
            # STOP the robot
            self.pub_cmd.publish(Twist())

            # Deactivate this node
            self.active = False

            # Switch the global state to sweep
            self.pub_state.publish(0)
            return
        

        self.pub_cmd.publish(move)

if __name__ == '__main__':
    rospy.init_node('roadless_PID_node')
    try:
        follower = RoadlessFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass