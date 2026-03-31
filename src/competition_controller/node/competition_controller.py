#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock

# Initialize score tracker node and publisher
rospy.init_node('topic_publisher')
pub_score = rospy.Publisher('/score_tracker', String, queue_size=1)

# Initialize state changer node
rospy.init_node('state_changer')
pub_state = rospy.Publisher('/state_changer', int, queue_size=1)
# 0 - plate_sweep
# 1 - road_PID

rospy.sleep(1)
pub_score.publish("looker,1111,0,aaaa") # Need to change message
pub_state.publish(1)

