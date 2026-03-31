#! /usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image

PLATE_SWEEP_NUMBER = 0

last_img = None
last_state = 1
images_list = []

# Needs to be global
pub_state = None

def callback_img(data):
    global last_img
    last_img = data

def callback_state(data):
    global pub_state
    global last_state
    if data.data == PLATE_SWEEP_NUMBER:
        if last_img is not None:
            images_list.append(last_img)
        pub_state.publish(last_state)
    else:
        last_state = data.data

def main():
    global pub_state
    rospy.init_node('plate_sweep_node', anonymous=True)

    # Initialize publishers and subscribers
    pub_state = rospy.Publisher('/state_changer', Int32, queue_size=1, latch=True)
    rospy.Subscriber('/B1/pi_camera/image_raw', Image, callback_img, queue_size=1)
    rospy.Subscriber('/state_changer', Int32, callback_state)
    rospy.spin()

if __name__ == '__main__':
    main()