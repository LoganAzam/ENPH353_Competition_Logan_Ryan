#! /usr/bin/env python3

import rospy
from std_msgs.msg import Int32

PLATE_SWEEP_NUMBER = 0

# Controls if node is running or not
node_on = False

def callback(data):
    if(data.data == PLATE_SWEEP_NUMBER):
        node_on = True
    else:
        node_on = False

# Listen to state changer topic
rospy.init_node('plate_sweep_node', anonymous=True)
rospy.Subscriber('/state_changer', Int32, callback)

while not rospy.is_shutdown():
    if node_on:
        # Do things
        break