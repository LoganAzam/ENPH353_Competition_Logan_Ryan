#! /usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class DirtroadVisualizer:
    def __init__(self):
        self.bridge = CvBridge()
        self.active = True 
    
        self.image_sub = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.callback, queue_size=1)

        rospy.loginfo("Visualizer Node Initialized. Press 'q' to exit.")

    def callback(self, data):
        if not self.active:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return

        # 1. Setup Dimensions
        h, w, _ = cv_image.shape
        
        # --- UPDATED ROI BOUNDARIES TO MATCH YOUR LINE ---
        # Top: shifted up slightly by h/16
        y_start = int(h/2) - int(h/16) 
        # Bottom: 80% down the screen
        y_end = int(4*h/5)
        # Left: Vertical centerline
        x_start = int(w/2)
        # Right: Right edge
        x_end = int(w)

        # 2. Convert to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 3. Create the Blue Mask
        lower_blue = np.array([100, 120, 30])
        upper_blue = np.array([140, 255, 255])
        blue_mask_full = cv2.inRange(hsv, lower_blue, upper_blue)

        # 4. Apply the restricted ROI
        # Create blank mask and copy only the ROI pixels
        mask_roi_only = np.zeros_like(blue_mask_full)
        mask_roi_only[y_start:y_end, x_start:x_end] = blue_mask_full[y_start:y_end, x_start:x_end]

        # 5. Count pixels ONLY in that specific window
        pixel_count = cv2.countNonZero(mask_roi_only)

        # 6. Visual Feedback
        # The green box now shows exactly what your 'blue_bottom_half' slice covers
        cv2.rectangle(cv_image, (x_start, y_start), (x_end, y_end), (0, 255, 0), 2)
        
        cv2.putText(cv_image, f"Pixels in Slice: {pixel_count}", 
                    (x_start + 5, y_start - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # 7. Visualization
        mask_bgr = cv2.cvtColor(mask_roi_only, cv2.COLOR_GRAY2BGR)
        stacked_view = np.hstack((cv_image, mask_bgr))

        cv2.imshow("Blue ROI Match Test", stacked_view)
        
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