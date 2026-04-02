#! /usr/bin/env python3

from competition_controller.node.road_PID import RoadFollower
import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int32  
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class dirtroad_follower:
    def __init__(self):
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

        # Mandatory 1-second delay for ROS Master registration 
        rospy.sleep(1)
    
    def state_callback(self, msg):
        if msg.data == 3:
            if not self.active:
                rospy.loginfo("dirtroad_PID node activated.")
            self.active = True
        else:
            if self.active:
                rospy.loginfo("dirtroad_PID node deactivated.")
            self.active = False

    if __name__ == '__main__':
    # Initialize the ROS node
        rospy.init_node('dirtroad_PID_node')

    try:
        follower = dirtroad_follower()
        # Keep the node running until interrupted
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

