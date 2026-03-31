#! /usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String  # cite: 98
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

#state_sub = rospy.Subscriber('/B1/state', String, state_callback) # cite: 98

class LineFollower:
    def __init__(self):
        self.bridge = CvBridge()
    
        self.pub_cmd = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
        self.pub_score = rospy.Publisher('/score_tracker', String, queue_size=1)
        
        # Corrected Topic: Based on your XML <cameraName> and <imageTopicName>
        self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.callback, queue_size=3)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # 1. Convert to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 2. Define Grey Road Range
        # Hue: 0-180 (Grey doesn't have a specific hue)
        # Saturation: 0-50 (LOW saturation isolates greyscale/grey)
        # Value: 50-200 (Ignore very dark shadows or very bright reflections)
        lower_grey = np.array([0, 0, 50]) 
        upper_grey = np.array([180, 50, 200])
        
        # Create mask for grey road
        mask = cv2.inRange(hsv, lower_grey, upper_grey)

        cv2.imshow("Centroid View", mask) # Debug: Show the mask to verify it's capturing the road
        cv2.waitKey(1)

        # 3. Focus on the road (Region of Interest)
        h, w, d = cv_image.shape
        search_top = int(3*h/4) # Look at the bottom quarter of the screen
        mask[0:search_top, 0:w] = 0

        # Calculate moments
        moments = cv2.moments(mask)
        move = Twist()

        if moments['m00'] > 0:
            cx = int(moments['m10']/moments['m00'])
            width = cv_image.shape[1]
            error = cx - width/2
            
            move.linear.x = 0.2
            move.angular.z = -float(error) / 100 # Adjusted sensitivity
        else:
            move.linear.x = 0
            move.angular.z = 0.3

        self.pub_cmd.publish(move) # Fixed: matched the attribute name in __init__

if __name__ == '__main__' and :
    rospy.init_node('line_follower_node')
    try:
        follower = LineFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass