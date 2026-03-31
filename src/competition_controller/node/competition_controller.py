#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
# from rospy import Time

rospy.init_node('topic_publisher')
pub_score = rospy.Publisher('/score_tracker', String, queue_size=1)

# Code below to do things before timer start

rospy.sleep(1)
pub_score.publish("looker,1111,0,aaaa") # Need to change message

