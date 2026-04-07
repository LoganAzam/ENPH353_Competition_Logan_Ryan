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
        
        # --- ROI BOUNDARIES: BOTTOM 60% ---
        # We skip the top 40% (0.4) to keep the bottom 60%
        y_start = int(h * 0.4)
        y_end = h
        x_start = 0
        x_end = w

        # 2. Convert to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 3. Create the Blue Mask
        lower_blue = np.array([100, 120, 30])
        upper_blue = np.array([140, 255, 255])
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # 4. Apply the restricted ROI
        mask_roi_only = np.zeros_like(blue_mask)
        mask_roi_only[y_start:y_end, x_start:x_end] = blue_mask[y_start:y_end, x_start:x_end]

        # 5. Calculate Moments and Centroid (The Homing Point)
        moments = cv2.moments(mask_roi_only)
        cx, cy = None, None

        if moments['m00'] > 0:
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])

        # 6. Visualization Prep
        # Convert mask to BGR so we can draw a RED dot on it
        mask_bgr = cv2.cvtColor(mask_roi_only, cv2.COLOR_GRAY2BGR)

        if cx is not None and cy is not None:
            # Draw RED dot on both images
            cv2.circle(mask_bgr, (cx, cy), 10, (0, 0, 255), -1)
            cv2.circle(cv_image, (cx, cy), 10, (0, 0, 255), -1)

        # 7. Visual Feedback
        # Green box highlights the Bottom 60% ROI
        cv2.rectangle(cv_image, (x_start, y_start), (x_end - 1, y_end - 1), (0, 255, 0), 2)
        
        pixel_count = cv2.countNonZero(mask_roi_only)
        cv2.putText(cv_image, f"Blue Pixels: {pixel_count}", 
                    (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 8. Display
        stacked_view = np.hstack((cv_image, mask_bgr))
        cv2.imshow("Blue Sign Debug (Bottom 60%)", stacked_view)
        
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