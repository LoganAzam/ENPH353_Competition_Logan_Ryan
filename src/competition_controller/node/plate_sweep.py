#! /usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
import os

PLATE_SWEEP_NUMBER = 0

last_state = 1
last_img = None
images_list = []

# Needs to be global
pub_state = None
pub_clue = None

def callback_img(data):
    global last_img
    last_img = data

def callback_state(data):
    global pub_state
    global last_state
    if data.data == PLATE_SWEEP_NUMBER:
        print("Sweep state entered") # DEBUG
        if last_img is not None:
            pub_clue.publish(last_img)
            # images_list.append(last_img.copy())

            # # Save Image
            # if not os.path.exists("saved_images"):
            #     os.makedirs("saved_images")
            # count = len(images_list)  # sequential number
            # filename = os.path.join("saved_images", f"image_{count:03d}.png")
            # cv2.imwrite(filename, last_img.copy())
            # print("IMG Saved") # DEBUG
            # rospy.sleep(1) # Give time for the image to save before changing state

        pub_state.publish(last_state)
    else:
        last_state = data.data

def main():
    global pub_state
    global pub_clue
    rospy.init_node('plate_sweep_node', anonymous=True)

    # Initialize publishers and subscribers
    pub_state = rospy.Publisher('/state_changer', Int32, queue_size=1, latch=True)
    pub_clue = rospy.Publisher('/clue_images', Image, queue_size=1)
    rospy.Subscriber('/B1/rrbot/camera1/image_raw', Image, callback_img, queue_size=1)
    rospy.Subscriber('/state_changer', Int32, callback_state)
    rospy.spin()

if __name__ == '__main__':
    main()