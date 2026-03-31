#! /usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int32  # cite: 98
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
    
        # Mandatory publishers for the competition [cite: 91]
        self.pub_cmd = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
        self.pub_score = rospy.Publisher('/score_tracker', String, queue_size=1)
        
        # Correct Topic: Based on your XML <cameraName> and <imageTopicName>
        self.image_sub = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.callback, queue_size=3)
        
        # Subscriber to manage the modular state of the robot
        self.state_sub = rospy.Subscriber('/state_changer', Int32, self.state_callback, queue_size=1)

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

        # 1. Convert to HSV to isolate color independent of brightness [cite: 323]
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 2. Define Grey Road Range (Low saturation isolates grey from green/blue)
        # Hue: 0-180, Saturation: 0-50 (Low), Value: 50-200 (Mid-range brightness)
        lower_grey = np.array([0, 0, 50]) 
        upper_grey = np.array([180, 50, 200])
        
        # Create a binary mask where grey road pixels are white (255)
        mask = cv2.inRange(hsv, lower_grey, upper_grey)
        rospy.loginfo("Mask created with grey range: Hue(0-180), Sat(0-50), Val(50-200)")
        

        # 3. Define Region of Interest (ROI)
        # We only look at the bottom quarter of the image to stay focused on the road
        h, w, _ = cv_image.shape
        search_top = int(3 * h / 3)
        mask[0:search_top, 0:w] = 0

        # 4. Calculate Moments to find the road's center [cite: 324]
        moments = cv2.moments(mask)
        move = Twist()

        if moments['m00'] > 0:
            # Calculate the horizontal center (centroid)
            cx = int(moments['m10'] / moments['m00'])
            
            # Calculate steering error (distance from image center)
            error = cx - w / 2
            
            # P-Controller Logic
            move.linear.x = 0.2  # Constant forward velocity
            move.angular.z = -float(error) / 100  # Proportional steering
        else:
            # If the road is lost, rotate slowly to find it
            move.linear.x = 0
            move.angular.z = 0.3

        # Publish movement command to the simulation [cite: 91]
        self.pub_cmd.publish(move)

        # Optional Debug Window (comment out for final run to save RTF) [cite: 297]
        cv2.imshow("Grey Road Mask", mask)
        cv2.waitKey(1)

if __name__ == '__main__':
    # Initialize the ROS node [cite: 66, 208]
    rospy.init_node('road_PID_node')
    
    try:
        follower = RoadFollower()
        # Keep the node running until interrupted [cite: 219]
        rospy.spin()
    except rospy.ROSInterruptException:
        pass