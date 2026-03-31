#! /usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image

PLATE_SWEEP_NUMBER = 0
images_list = []

def callback_state(data):
    if data.data == PLATE_SWEEP_NUMBER:
        # Start subscribing to camera topic.
        rospy.Subscriber('/rrbot/camera1/image_raw', Image, callback_img)

def plate_sweep_listener():
    # Listen to state changer topic
    rospy.init_node('plate_sweep_node', anonymous=True)
    rospy.Subscriber('/state_changer', Int32, callback_state)
    rospy.spin()

if __name__ == '__main__':
    plate_sweep_listener()