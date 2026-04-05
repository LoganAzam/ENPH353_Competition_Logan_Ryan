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

        rospy.sleep(1)

    def state_callback(self, msg):
        # State 5 is for Roadless
        if msg.data == 5:
            if not self.active:
                rospy.loginfo("roadless_PID node activated.")
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

        # Process the image for roadless detection
        # ...
