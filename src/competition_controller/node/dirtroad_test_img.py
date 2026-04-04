#! /usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class DirtroadVisualizer:
    def __init__(self):
        self.bridge = CvBridge()
        self.active = True  # Set to True by default for easier testing
    
        # Subscriber to the camera
        self.image_sub = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.callback, queue_size=1)

        rospy.loginfo("Visualizer Node Initialized. Press 'q' in the window to exit.")

    def callback(self, data):
        if not self.active:
            return

        try:
            # Convert ROS Image to OpenCV BGR
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return

        # 1. Setup Dimensions and ROI coordinates
        h, w, _ = cv_image.shape
        search_top = int(0.65 * h)
        search_bottom = h
        search_left = 0
        search_right = int(w/2) # Currently set to left half based on your script

        # 2. Convert to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 3. Create the Mask (Tune these values!)
        lower_white = np.array([25, 0, 174])
        upper_white = np.array([65, 76, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # 4. Apply the ROI to the Mask 
        # We black out everything OUTSIDE the bounding box
        roi_mask = np.zeros_like(mask)
        roi_mask[search_top:search_bottom, search_left:search_right] = \
            mask[search_top:search_bottom, search_left:search_right]

        # 5. Draw the Bounding Box on the color image
        # cv2.rectangle(image, top_left, bottom_right, color(BGR), thickness)
        cv2.rectangle(cv_image, (search_left, search_top), (search_right, search_bottom), (0, 255, 0), 3)
        
        # Add a text label to the box
        cv2.putText(cv_image, "ROI - Dirt Road Search", (search_left + 10, search_top - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 6. Visualization
        # Convert mask to BGR so we can stack it with the color image
        mask_bgr = cv2.cvtColor(roi_mask, cv2.COLOR_GRAY2BGR)
        stacked_view = np.hstack((cv_image, mask_bgr))

        cv2.imshow("Dirt Road Debug (Left: ROI Box | Right: Resulting Mask)", stacked_view)
        
        # Keep waitKey to allow the window to refresh
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("User requested quit")

if __name__ == '__main__':
    rospy.init_node('dirtroad_visualizer_node')
    try:
        visualizer = DirtroadVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()