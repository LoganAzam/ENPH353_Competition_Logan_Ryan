#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

rospy.init_node('topic_publisher')
pub_cmd = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
pub_score = rospy.Publisher('/score_tracker', String, queue_size=1)
rate = rospy.Rate(2)
move = Twist()
move.linear.x = 0.5
move.angular.z = -0.1

#while not rospy.is_shutdown():
rospy.sleep(1)
pub_score.publish("looker,1111,0,aaaa")
rospy.sleep(1)
pub_cmd.publish(move)
pub_score.publish("looker,1111,-1,aaaa")

   #rate.sleep()

